#ifndef linkLL_H
#define  linkLL_H
/* ------------------ basic link stuff ----------------------------------*/
/*  original link-node code from

    Anasazi Linked List Utility Routines - 1.15
    by Duane Morse     e-mail: duane@anasaz (or ... asuvax!anasaz!duane)

    modified end extended by
        G. Matas   (g.matas@ee.surrey.ac.uk)
*/
#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {  /* can be included directly into C++ progs */
#endif

#define L_BASIC_MACRO 1         /* implement l_nextl, l_prevl as macros */

#ifdef _MSC_VER
#define inline __inline
#endif

#ifdef L_BASIC_MACRO

#define l_nextl(link) ((link)->forward)
#define l_prevl(link) ((link)->backward)
#define l_lempty(link) (((link) == ((link)->forward)))

#else

    static inline l_list *l_nextl(l_list *link)
    {
        return link->forward;
    }
    static inline l_list *l_prevl(l_list *link)
    {
        return link->backward;
    }
    static inline int    l_lempty(l_list *link)
    {
        return (link->forward == link);
    }
#endif

    /*--------------- linking ------------------------------------------*/
    static inline void l_lafter( l_list *current, l_list *newEl)
    {
        newEl->forward = current->forward;
        newEl->backward = current;
        current->forward->backward = newEl;
        current->forward = newEl;
    }

    static inline void l_lbefore(l_list *current,l_list  *newEl)
    {
        newEl->forward = current;
        newEl->backward = current->backward;
        current->backward->forward = newEl;
        current->backward = newEl;
    }

    static inline void l_unlink(l_list *link)
    {
        link->forward->backward = link->backward;
        link->backward->forward = link->forward;
    }

    static inline void l_linit(l_list *link)
    {
        link->forward = link->backward = link;
    }

#define ForeachLink_M(head,link)\
     for(link=l_nextl(head); head != link; link=l_nextl(link))
#define ForeachLinkBack_M(head,link)\
     for(link=l_prevl(head); head != link; link=l_prevl(link))

    /*--------------------------------------------------------------------------*/

    /* conversion link - LL */
#define elm2link(el)  ((l_list*)((t_linkLL *)(el) - 1))
#define link2elm(li)  ((void *) ((t_linkLL *)(li) + 1))
#define list2link(list) (&((list)->links.u.ll))

#if defined(__cplusplus) || defined(c_plusplus)
}
#endif

#endif

