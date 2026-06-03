"""Minimal heapq for CircuitPython (min-heap, heappush + heappop only)."""


def heappush(heap, item):
    heap.append(item)
    _sift_up(heap, len(heap) - 1)


def heappop(heap):
    last = heap.pop()
    if heap:
        root = heap[0]
        heap[0] = last
        _sift_down(heap, 0)
        return root
    return last


def _sift_up(heap, pos):
    item = heap[pos]
    while pos > 0:
        parent = (pos - 1) >> 1
        if item < heap[parent]:
            heap[pos] = heap[parent]
            pos = parent
        else:
            break
    heap[pos] = item


def _sift_down(heap, pos):
    n = len(heap)
    item = heap[pos]
    while True:
        child = 2 * pos + 1
        if child >= n:
            break
        right = child + 1
        if right < n and heap[right] < heap[child]:
            child = right
        if heap[child] < item:
            heap[pos] = heap[child]
            pos = child
        else:
            break
    heap[pos] = item
