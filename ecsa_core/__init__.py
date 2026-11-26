"""Core of the event-camera collision-urgency dodge (ECSA-13), free of ROS and simulator code."""
from .params import Params
from .loop import EventFront, Avoider, run_episode
