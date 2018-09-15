#include "queue.h"

void init_queue(Queue &q)
{
	q.front = q.rear = 0;
	//pthread_mutex_init(&mutex_queue, NULL);
}

void des_queue(Queue &q)
{
	//pthread_mutex_destroy(&mutex_queue);
}

void in_queue(Queue &q)
{
#pragma omp critical (queue_section)
	{
		if (((q.rear + 1) % QUEUE_MAX_SIZE) == q.front ? 0 : 1)
		{
			q.rear = (q.rear + 1) % QUEUE_MAX_SIZE;
		}
	}
	
}

void out_queue(Queue &q)
{
#pragma omp critical (queue_section)
	{
		if ((q.rear == q.front) ? 0 : 1)
		{
			q.front = (q.front + 1) % QUEUE_MAX_SIZE;
		}
	}
	
}

int isQueueEmpty(Queue &q)
{
	int res;

#pragma omp critical (queue_section)
	{
		res = (q.rear == q.front) ? 1 : 0;
	}
	

	return res;
}

int isQueueFull(Queue &q)
{
	int res;

#pragma omp critical (queue_section)
	{
		res = ((q.rear + 1) % QUEUE_MAX_SIZE) == q.front ? 1 : 0;
	}
	
	return res;
}

int nextQueueRear(Queue &q)
{
	int next;

#pragma omp critical (queue_section)
	{
		next = (q.rear + 1) % QUEUE_MAX_SIZE;
	}

	return next;
}

int currentQueueFront(Queue &q)
{
	int current;

#pragma omp critical (queue_section)
	{
		current = (q.front + 1) % QUEUE_MAX_SIZE;
	}

	return current;
}