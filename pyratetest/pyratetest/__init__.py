"""
Simple example PubSub to measure communication rate.
Interestingly, either shared memory does not work for Python, 
or shared memory is somehow disabled, based on the stats.

Max Rate:
@ 1 char: ~12500Hz
@ 1,000,000 chars: ~3000Hz
@ 10,000,000 chars: ~60Hz (sudden exponential drop?)
"""
