"""

Dijkstra grid based planning

author: Atsushi Sakai
"""


class Node:

    def __init__(self):
        self.x = -1
        self.y = -1
        self.cost = 0.0
        self.pind = -1


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 50.0  # [m]
    sy = 450.0  # [m]
    gx = -50.0  # [m]
    gy = 560.0  # [m]

    ox = []
    oy = []

    #  for i in 0:
    #  60
    #  push!(ox, Float64(i))
    #  push!(oy, 0.0)
    #  end
    #  for i in 0:
    #  60
    #  push!(ox, 60.0)
    #  push!(oy, Float64(i))
    #  end
    #  for i in 0:
    #  60
    #  push!(ox, Float64(i))
    #  push!(oy, 60.0)
    #  end
    #  for i in 0:
    #  60
    #  push!(ox, 0.0)
    #  push!(oy, Float64(i))
    #  end
    #  for i in 0:
    #  40
    #  push!(ox, 20.0)
    #  push!(oy, Float64(i))
    #  end
    #  for i in 0:
    #  40
    #  push!(ox, 40.0)
    #  push!(oy, 60.0 - Float64(i))
    #  end


if __name__ == '__main__':
    main()
