/*
 * pci_dio24.h
 *
 *  Created on: 8 Dec 2016
 *      Author: cw66
 */

#ifndef PCI_DIO24_H_
#define PCI_DIO24_H_

int read_dio(comedi_t*, int, int, int);
int write_dio(comedi_t*, int, char*, int, int);



#endif /* PCI_DIO24_H_ */
