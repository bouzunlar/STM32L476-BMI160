#ifndef INC_SLAB_H_
#define INC_SLAB_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SLAB_SMALL_SIZE    128u
#define SLAB_LARGE_SIZE   2048u

#define SLAB_SMALL_COUNT     4u
#define SLAB_LARGE_COUNT     6u

void  Slab_Init (void);
void* Slab_Alloc(size_t size);
void  Slab_Free (void *ptr);

#ifdef __cplusplus
}
#endif

#endif /* INC_SLAB_H_ */
