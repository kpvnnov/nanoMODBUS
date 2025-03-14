#include "../../../../nanomodbus.h"

// set counter received symbols
void msg_buf_set(nmbs_t*, uint32_t);

// read counter received symbols
uint32_t msg_buf_get(nmbs_t *);

/** reset counter receved bytes
 * @param data Data
 * @param length Length of the data
 */
inline void msg_rec_reset(nmbs_t *nmbs){ msg_buf_set(nmbs, 0);}

