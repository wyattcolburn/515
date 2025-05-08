#ifndef SPI_H
#define SPI_H

void SPI_init(void);
void SPI_Master_Init(void);
void SPI_Slave_Init(void);
uint16_t SPI_Read_From_Peer(void);

typedef struct {
  uint16_t header; //size of the packet, type of data being sent?
  uint16_t data[MAX_DATA_SIZE];

} SPI_Packet;

#endif
