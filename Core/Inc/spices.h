/*
 * spices.h
 *
 *  Created on: Jul 19, 2021
 *      Author: lastl
 */

#ifndef INC_SPICES_H_
#define INC_SPICES_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define NUMCONTAINERS 7

typedef struct Spice {
	int container;	// Which container is the spice in
	char name[25];	// User-given spice name
	int dispensed;	// Amount dispensed in multiples of 1/4 tsp
} spice_t;

extern spice_t spice_list[NUMCONTAINERS];	// Global spice list variable

void initList(spice_t l[]);
bool addSpice(spice_t l[], int c, char n[]);
bool removeSpice(spice_t l[], int c);
void printSpices(spice_t l[]);
void sortName(spice_t l[]);
void sortContainer(spice_t l[]);
bool refillSpice(spice_t l[], int c);
int dispenseSpice(spice_t l[], int c, int a);

#endif /* INC_SPICES_H_ */
