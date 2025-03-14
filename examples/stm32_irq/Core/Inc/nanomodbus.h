
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __mynanomodbus_H__
#define __mynanomodbus_H__

#include "../../../../nanomodbus.h"

// set counter received symbols
void msg_buf_set(nmbs_t*, uint32_t);

// read counter received symbols
uint32_t msg_buf_get(nmbs_t *);

// incremet counter received symbols
bool msg_buf_inc(nmbs_t *nmbs);


/** reset counter receved bytes
 * @param data Data
 * @param length Length of the data
 */
inline void msg_rec_reset(nmbs_t *nmbs){ msg_buf_set(nmbs, 0);}

#endif /* __mynanomodbus_H__ */
