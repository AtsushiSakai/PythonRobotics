import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="A-star",
    version="0.0.1",
    author="Adwait Naik",
    author_email="adwaitnaik2@gmail.com",
    description="Python implemenation of the A-star algorithm",
    long_description="A* (pronounced as "A star") is a computer algorithm that is widely used in pathfinding and graph traversal. The algorithm efficiently plots a walkable path between multiple nodes, or points, on the graph. A non-efficient way to find a path. On a map with many obstacles, pathfinding from points A to B can be difficult.",
    long_description_content_type="text/markdown",
    url="https://github.com/addy1997/Astar",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
