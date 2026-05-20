#ifndef INC_INTERRUPT_H_
#define INC_INTERRUPT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef enum
{
	INTERRUPT_ID_BMI160_INT1,
	INTERRUPT_ID_MAX
} interrupt_id_t;

typedef enum
{
	INTERRUPT_ERR_NONE,
	INTERRUPT_ERR_INVALID_ID,
	INTERRUPT_ERR_NULL_HANDLER
} interrupt_err_t;

typedef void (*interrupt_handler_t)(void *context);

void            Interrupt_Init       (void);
interrupt_err_t Interrupt_Register   (interrupt_id_t id, interrupt_handler_t handler, void *context);
interrupt_err_t Interrupt_Unregister (interrupt_id_t id);
void            Interrupt_Dispatch   (interrupt_id_t id);

uint32_t        Interrupt_GetCount   (interrupt_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* INC_INTERRUPT_H_ */
