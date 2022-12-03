"""
Simple example PubSub to measure communication rate. Surprisingly, the RQT
topic monitor plugin chokes if the messages are:
- big (like images)
- very rapid
- published using Best Effort (vs Reliable) QoS
and fails to display the rate or monitor the topic contents. Hence this example 
as a potential replacement.

Also, shared memory only works if using C++. Sorry.
See: https://github.com/ros2/design/issues/251 on progress for bring shared
memory to all language clients (stalled)

Max Rate:
@ 1 char: ~12500Hz
@ 1,000,000 chars: ~3000Hz
@ 10,000,000 chars: ~60Hz (sudden exponential drop?)
"""
