#ifndef PHOENIX_VM
#define PHOENIX_VM

struct vm_area_struct{
    void *dummy;
    unsigned long vm_start;
    unsigned long vm_end;
    int vm_page_prot;
};

#endif