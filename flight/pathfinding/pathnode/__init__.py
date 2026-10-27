"""Path-node planner: a from-scratch replacement for the maze A/B/C replanner.

The route is one ordered chain of path nodes from the bottom entry to the top entry.
Consecutive nodes are joined by a link that is either checked (segment C), pending
(segment B, claimed by one drone) or the single forward gap between the two sides'
point As. See planner.py for the rules.

Deliberately independent of the old planner: nothing here imports flight.pathfinder,
flight.pathfinding.nodeField, protoMine or path_calculation. The only shared pieces are
the coverage primitives -- CellField (a bitset grid) and path_cover_unseen (camera
placement) -- kept so photo counts stay comparable with the old simulation.

Layers: session.py is PathNodeBackend, this package's SearchBackend (the contract in
flight/pathfinding/search_backend.py); planner.py is the planner it wraps; sim.py drives any
registered backend.
"""
