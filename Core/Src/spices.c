/*
 * spices.c
 *
 *  Created on: Jul 19, 2021
 *      Author: lastl
 */

#include "spices.h"

/**
 * Initialize the allocated memory block to ensure no random
 * data in memory causes reading errors
 */
void initList(spice_t l[]) {
	for (int i = 0; i < NUMCONTAINERS; i++) {
//		spice_t temp;
//		temp.name[0] = 0;
//		temp.container = -1;
//		temp.dispensed = -1;
//		l[i] = temp;
		l[i].container = -1;
		l[i].dispensed = -1;
		memset(l[i].name, 0, sizeof(l[i].name));
	}
}

/**
 * Add a spice to the list; if container is already
 * in use, replace that spice
 */
bool addSpice(spice_t l[], int c, char n[]) {
	// Error checking
	if (c >= NUMCONTAINERS || c < 0) {
		printf("CONTAINER OUT OF BOUNDS");
		return false;
	}
	spice_t temp;
	temp.container = c;
	temp.dispensed = 0;
	snprintf(temp.name, 25, "%s", n);	// Copy string safely

	// Check for existing spice in given container
	int i;
	for (i = 0; i < NUMCONTAINERS; i++) {
		if (l[i].container == c) {
//			l[i] = temp;
			memset(l[i].name, 0, sizeof(l[i].name));
			l[i].container = c;
			l[i].dispensed = 0;
			snprintf(l[i].name, 25, "%s", n);	// Safe string copy
			return true;
		}
	}

	// If container not being used, put in empty array index
	for (i = 0; i < NUMCONTAINERS; i++) {
		if (l[i].container == -1) {
//			l[i] = temp;
			l[i].container = c;
			l[i].dispensed = 0;
			snprintf(l[i].name, 25, "%s", n);	// Safe string copy
			return true;
		}
	}

	// If you made it this far, something went wrong
	return false;
}

/**
 * Remove a spice from the list
 */
bool removeSpice(spice_t l[], int c) {
	// Error checking
	if (c >= NUMCONTAINERS || c < 0) {
		printf("CONTAINER OUT OF BOUNDS\n");
		return false;
	}
	// Find spice record for container c
	int i;
	for (i = 0; i < NUMCONTAINERS; i++) {
		if (l[i].container == c) {
//			l[i].name[0] = 0;
			memset(l[i].name, 0, sizeof(l[i].name));
			l[i].container = -1;
			l[i].dispensed = -1;
			return true;
		}
	}
	printf("No spice found for container %d\n", c);
	return true;
}

int dispenseSpice(spice_t l[], int c, int a) {
	// Error checking
	if (c >= NUMCONTAINERS || c < 0) {
		printf("CONTAINER OUT OF BOUNDS\n");
		return -1;
	}
	// Find spice record for container c
	int i;
	for (i = 0; i < NUMCONTAINERS; i++) {
		if (l[i].container == c) {
			l[i].dispensed += a;
			return l[i].dispensed;
		}
	}
	printf("No spice found for container %d\n", c);
	return -1;
}

bool refillSpice(spice_t l[], int c) {
	// Error checking
	if (c >= NUMCONTAINERS || c < 0) {
		printf("CONTAINER OUT OF BOUNDS\n");
		return false;
	}
	// Find spice record for container c
	int i;
	for (i = 0; i < NUMCONTAINERS; i++) {
		if (l[i].container == c) {
			l[i].dispensed = 0;
			return true;
		}
	}
	printf("No spice found for container %d\n", c);
	return true;
}

/**
 * Print spices in the list; mostly for debugging
 */
void printSpices(spice_t l[]) {
	for (int i = 0; i < NUMCONTAINERS; i++) {
		if (strcmp(l[i].name, "") != 0) {
			printf("%d: %s\n", l[i].container, l[i].name);
		}
	}
	printf("\n");
}

/**
 * Sort the spices alphabetically
 */
void sortName(spice_t l[]) {
	spice_t temp;
	int i, j;
	for (i = 0; i < NUMCONTAINERS; i++) {
		for (j = i + 1; j < NUMCONTAINERS; j++) {
			if (strcmp(l[i].name, l[j].name) > 0) {
				temp = l[i];
				l[i] = l[j];
				l[j] = temp;
			}
		}
	}
}

/**
 * Sort the spices by container number
 */
void sortContainer(spice_t l[]) {
	spice_t temp;
	int i, j;
	for (i = 0; i < NUMCONTAINERS; i++) {
		for (j = i + 1; j < NUMCONTAINERS; j++) {
			if (l[j].container < l[i].container) {
				temp = l[i];
				l[i] = l[j];
				l[j] = temp;
			}
		}
	}
}
