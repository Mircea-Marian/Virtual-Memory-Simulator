/*
 * 332CB
 * Popa Mirezea-Marian
 */

#include "common.h"
#include "vmsim.h"
#include "vmsim_aux.h"
#include "helpers.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "util.h"

/* Aici se retin datele despre simulari. */
static simpleLinkedList_t *dataList;

/* Se elibereaza resursele folosite de simulator. */
void freeResourezes(simpleLinkedList_t *dataList, w_size_t page_size)
{
	free(dataList->data.frame_list);
	free(dataList->data.page_table);

	munmap(dataList->data.SWAP_addr_start,
	       dataList->data.g_num_pages * page_size);
	munmap(dataList->data.RAM_addr_start,
	       dataList->data.g_num_frames * page_size);
	munmap(dataList->data.page_table->start,
	       dataList->data.g_num_pages * page_size);

	close(dataList->data.RAM_file);
	close(dataList->data.SWAP_file);

	free(dataList);
}

/* Se implementeaza tratarea semnalului de SIGSEGV. */
void signal_handler(int signum, siginfo_t *info, void *context)
{

	unsigned int i;
	struct page_table_entry *page_entry_addr;
	const w_size_t page_size = w_get_page_size();
	const w_size_t offset = (unsigned long)info->si_addr % page_size;
	void *fault_page_addr = info->si_addr - offset, *rez;
	struct frame *frame_addr;
	void *p;
	simpleLinkedList_t *iterator;
	int rc;

	if (signum == SIGSEGV) {
		/* Se gaseste memoria unde este alocata adresa. */
		iterator = dataList;
		while (iterator &&
		       !(iterator->data.page_table->start <= fault_page_addr &&
			 fault_page_addr <= iterator->data.page_table->start +
			 (iterator->data.g_num_pages - 1) * page_size))
			iterator = iterator->next;

		/*
		 * Se gaseste entry-ul in tabela de pagini pentru pagina ce a
		 * determinat segmentation fault.
		 */
		page_entry_addr = iterator->data.page_table;
		i = iterator->data.g_num_pages;
		while (i-- && page_entry_addr->start != fault_page_addr)
			page_entry_addr++;

		if (page_entry_addr->state == STATE_NOT_ALLOC) {
			// Se aloca pagina in RAM.

			// Se itereaza prin paginile din RAM.
			frame_addr = iterator->data.frame_list;
			i = iterator->data.g_num_frames;
			while (i-- && frame_addr->pte != NULL)
				frame_addr++;

			//Se implementeaza swap out.
			if (frame_addr ==
			    iterator->data.frame_list +
			    iterator->data.g_num_frames) {

				while (frame_addr->pte != NULL)
					frame_addr++;

				// Se copiaza continutul in swap, din prima
				// pagina din RAM.
				if (page_entry_addr->dirty == TRUE
				    || page_entry_addr->prev_state ==
				    STATE_NOT_ALLOC)
					memcpy(iterator->data.SWAP_addr_start +
					       frame_addr->index * page_size,
					       iterator->data.RAM_addr_start,
					       page_size);

				// Se modifica pagina din SWAP.
				frame_addr->pte =
				    iterator->data.frame_list->pte;

				// Se modifica entry-ul din tabela de pagini
				// pentru pagina ce trebuie scoasa din RAM.
				iterator->data.frame_list->pte->prev_state =
				    iterator->data.frame_list->pte->state;
				iterator->data.frame_list->pte->state =
				    STATE_IN_SWAP;
				iterator->data.frame_list->pte->frame =
				    frame_addr;

				// Se demapeaza pagina virtuala care era
				// asociata cu prima pagina dun RAM.
				rc = munmap(
					iterator->data.frame_list->pte->start,
				       page_size);
				DIE(rc == -1, "eroare memorie virtuala\n");
				rez = mmap(
				     iterator->data.frame_list->pte->start,
				     page_size, PROT_NONE,
				     MAP_SHARED | MAP_ANONYMOUS | MAP_FIXED, -1,
				     0);
				DIE(rez == MAP_FAILED,
					"eroare memorie virtuala\n");
				frame_addr = iterator->data.frame_list;
			}
			// Se "curata" pagina.
			bzero(iterator->data.RAM_addr_start +
			      frame_addr->index * page_size, page_size);

			// Se actualizeaza entry-ul din tabela.
			page_entry_addr->state = STATE_IN_RAM;
			page_entry_addr->dirty = FALSE;
			page_entry_addr->protection = PROTECTION_READ;
			page_entry_addr->frame = frame_addr;
			frame_addr->pte = page_entry_addr;

			rez = mmap(page_entry_addr->start, page_size, PROT_READ,
			     MAP_SHARED | MAP_FIXED, iterator->data.RAM_file,
			     frame_addr->index * page_size);
			DIE(rez == MAP_FAILED, "eroare memorie virtuala !\n");

		} else if (page_entry_addr->state == STATE_IN_RAM) {
			// Se modifica protectiile si dirty bit-ul.
			page_entry_addr->dirty = TRUE;
			page_entry_addr->protection = PROTECTION_WRITE;

			// Se actualizeaza protectiile.
			w_protect_mapping(page_entry_addr->start, 1,
					  PROTECTION_WRITE);
		} else if (page_entry_addr->state == STATE_IN_SWAP) {
			page_entry_addr->state = STATE_IN_RAM;

			// Se itereaza prin paginile din RAM.
			frame_addr = iterator->data.frame_list;
			i = iterator->data.g_num_frames;
			while (i-- && frame_addr->pte != NULL)
				frame_addr++;
			if (frame_addr !=
			    iterator->data.frame_list +
			    iterator->data.g_num_frames) {
				// Se gaseste o pagina libera in RAM.

				// Se copiaza continutul.
				memcpy(iterator->data.RAM_addr_start +
					frame_addr->index * page_size,
					iterator->data.SWAP_addr_start +
					page_entry_addr->frame->index *
					page_size, page_size);

				// Se reseteaza in pagina din SWAP.
				bzero(iterator->data.SWAP_addr_start +
				      page_entry_addr->frame->index * page_size,
				      page_size);

				// Se elibereaza frame-ul din SWAP.
				page_entry_addr->frame->pte = NULL;

				// Se face legatura dintre paginile din RAM si
				// SWAP.
				page_entry_addr->frame = frame_addr;
				frame_addr->pte = page_entry_addr;
			} else {
				// Se face exchange intre RAM si SWAP de o
				// pagina.
				p = malloc(page_size);
				DIE(!p, "eroare de alocare memorie !\n");

				memcpy(p, iterator->data.RAM_addr_start,
				       page_size);

				// Se face un schimb de continut din SWAP in
				// RAM.
				memcpy(iterator->data.RAM_addr_start,
				       iterator->data.SWAP_addr_start +
				       page_entry_addr->frame->index *
				       page_size, page_size);

				memcpy(iterator->data.SWAP_addr_start +
				       page_entry_addr->frame->index *
				       page_size, p, page_size);

				free(p);

				// Se fac actualizari in metadate.
				iterator->data.frame_list->pte->prev_state =
				    iterator->data.frame_list->pte->state;
				iterator->data.frame_list->pte->state =
				    STATE_IN_SWAP;
				iterator->data.frame_list->pte->frame =
				    page_entry_addr->frame;
				iterator->data.frame_list->pte->frame->pte =
				    iterator->data.frame_list->pte;

				// Se demapeaza prima pagina din fisierul de
				// RAM.
				rc = munmap(
					iterator->data.frame_list->pte->start,
				       page_size);
				DIE(rc == -1, "eroare memorie virtuala\n");
				rez = mmap(
				     iterator->data.frame_list->pte->start,
				     page_size, PROT_NONE,
				     MAP_SHARED | MAP_ANONYMOUS | MAP_FIXED,
				     -1,
				     0);
				DIE(rez == MAP_FAILED,
					"eroare memorie virtuala !\n");

				page_entry_addr->frame =
				    iterator->data.frame_list;
				iterator->data.frame_list->pte =
				    page_entry_addr;

				frame_addr = iterator->data.frame_list;
			}
			// Se mapeaza in pagina virtuala in pagina din RAM.
			page_entry_addr->protection = PROTECTION_READ;
			rez = mmap(page_entry_addr->start, page_size, PROT_READ,
			     MAP_SHARED | MAP_FIXED, iterator->data.RAM_file,
			     frame_addr->index * page_size);
			DIE(rez == MAP_FAILED, "eroare memorie virtuala !\n");
		}
	}
}

FUNC_DECL_PREFIX w_boolean_t vmsim_init(void)
{
	dataList = NULL;
	return w_set_exception_handler(signal_handler);
}

FUNC_DECL_PREFIX w_boolean_t vmsim_cleanup(void)
{
	w_exception_handler_t h;

	w_get_previous_exception_handler(&h);
	return w_set_exception_handler(h);
}

FUNC_DECL_PREFIX w_boolean_t vm_alloc(w_size_t num_pages, w_size_t num_frames,
				      vm_map_t *map)
{
	const w_size_t page_size = w_get_page_size();
	unsigned int i;
	struct page_table_entry *it;
	char ramName[] = "ramFileXXXXXX";
	char swapName[] = "swapFileXXXXXX";
	struct simpleLinkedList *iterator, *newCell;
	void *p;

	if (num_frames >= num_pages)
		return FALSE;

	// Daca nu a mai fost inceputa o simulare
	// se executa codul din if-ul urmator.
	if (!dataList) {
		dataList =
		    (struct simpleLinkedList *)
		    malloc(sizeof(struct simpleLinkedList));
		DIE(!dataList,
		    "Nu s-a putut aloca memorie pentru \"dataList\" !\n");
		dataList->next = NULL;
		iterator = dataList;
	} else {
		iterator = dataList;
		while (iterator->next)
			iterator = iterator->next;
		newCell =
		    (struct simpleLinkedList *)
		    malloc(sizeof(struct simpleLinkedList));
		DIE(!newCell,
		    "Nu s-a putut aloca memorie pentru \"newCell\" !\n");
		iterator->next = newCell;
		newCell->next = NULL;
		iterator = newCell;
	}

	iterator->data.g_num_frames = num_frames;
	iterator->data.g_num_pages = num_pages;

	// Se aloca paginile de memorie virtuala.
	map->start = mmap(NULL, num_pages * page_size, PROT_NONE,
			  MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	DIE(map->start == MAP_FAILED,
	    "Eroare memorie virtuala \"map->start\" !\n");

	// Se deschide fiserul de RAM si se mapeaza in memoria virtuala a
	// procesului.
	map->ram_handle = mkstemp(ramName);
	unlink(ramName);
	ftruncate(map->ram_handle, num_frames * page_size);
	iterator->data.RAM_file = map->ram_handle;
	iterator->data.RAM_addr_start = mmap(NULL, num_frames * page_size,
					     PROT_READ | PROT_WRITE, MAP_SHARED,
					     iterator->data.RAM_file, 0);
	DIE(iterator->data.RAM_addr_start == MAP_FAILED,
		"Eroare memorie virtula \"iterator->data.RAM_addr_start\" !\n");
	bzero(iterator->data.RAM_addr_start, num_frames * page_size);

	// Se deschide fiserul de SWAP si se mapeaza in memoria virtuala a
	// procesului.
	map->swap_handle = mkstemp(swapName);
	unlink(swapName);
	ftruncate(map->swap_handle, num_pages * page_size);
	iterator->data.SWAP_file = map->swap_handle;
	iterator->data.SWAP_addr_start =
	    mmap(NULL, num_pages * page_size, PROT_READ | PROT_WRITE,
		 MAP_SHARED, iterator->data.SWAP_file, 0);
	DIE(iterator->data.SWAP_addr_start == MAP_FAILED,
	    "eroare memorie virtuala\n");
	bzero(iterator->data.SWAP_addr_start, num_pages * page_size);

	// Se aloca metadata despre RAM.
	iterator->data.frame_list =
	    (struct frame *)malloc((num_frames + num_pages) *
				   sizeof(struct frame));
	DIE(iterator->data.frame_list == NULL,
		"eroare de alocare memorie !\n");

	// Se initializeaza metadata despre RAM.
	for (i = 0; i < num_frames; i++) {
		iterator->data.frame_list[i].index = i;
		iterator->data.frame_list[i].pte = NULL;
	}

	// Se initializeaza metadata despre SWAP.
	for (i = 0; i < num_pages; i++) {
		iterator->data.frame_list[i + num_frames].index = i;
		iterator->data.frame_list[i + num_frames].pte = NULL;
	}

	// Se aloca tabela de pagini.
	iterator->data.page_table =
	    (struct page_table_entry *)malloc(num_pages *
					      sizeof(struct page_table_entry));
	DIE(iterator->data.page_table == NULL,
		"eroare de alocare memorie !\n");

	// Se initializeaza tabela de pagini.
	i = num_pages;
	p = map->start;
	it = iterator->data.page_table;
	while (i--) {
		it->state = STATE_NOT_ALLOC;
		it->prev_state = STATE_NOT_ALLOC;
		it->dirty = FALSE;
		it->protection = PROTECTION_NONE;
		it->start = p;
		it->frame = NULL;

		it++;
		p += page_size;
	}

	return TRUE;
}

FUNC_DECL_PREFIX w_boolean_t vm_free(w_ptr_t start)
{

	const w_size_t page_size = w_get_page_size();
	struct simpleLinkedList *iterator, *trailingBehind;

	if (start == NULL)
		return FALSE;

	// Se elibereaza resursele daca doar o simulare este pornita.
	if (!dataList->next) {
		if (dataList->data.page_table->start == start) {
			freeResourezes(dataList, page_size);
			dataList = NULL;
			return TRUE;
		}
		return FALSE;
	}
	// Se elibereaza resursele daca mai multe simulari sunt pornite, iar
	// simularea de eliminat este in primul nod din lista.
	if (dataList->data.page_table->start == start) {
		iterator = dataList->next;

		freeResourezes(dataList, page_size);

		dataList = iterator;
		return TRUE;
	}
	// Se elibereaza resursa pentru
	trailingBehind = dataList;
	iterator = dataList->next;
	while (iterator && iterator->data.page_table->start != start)
		iterator = iterator->next;

	if (!iterator)
		return FALSE;

	// Se leaga lista precedenta de lista ulrmatoare dupa iterator.
	trailingBehind->next = iterator->next;

	// Se distruge iterator.
	freeResourezes(iterator, page_size);

	free(iterator);

	return TRUE;
}