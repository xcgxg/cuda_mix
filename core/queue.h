#pragma once

#include <omp.h>

#define QUEUE_MAX_SIZE 10

struct Queue_t
{
	int front;
	int rear;
};

typedef struct Queue_t Queue;

void init_queue(Queue &q);
void des_queue(Queue &q);
void in_queue(Queue &);
void out_queue(Queue &);
int isQueueEmpty(Queue &);
int isQueueFull(Queue &);
int nextQueueRear(Queue &q);
int currentQueueFront(Queue &q);