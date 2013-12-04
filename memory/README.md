ros-memory
==========

shared memory node for ROS. it consists of a memory server, providing ROS services for adding memory terms, removing memory terms, retrieving the log of operations, and retrieving the contents of memory.

add and delete operations can be performed directly from ROS services endpoints.

for notification of add/delete events, use the MemoryListener class.
