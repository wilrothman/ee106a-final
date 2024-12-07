`grid.py` is for discrete squares appearing in a grid, as opposed to the continuous
squares and circles seen in `game.py`.

Flags:
- **Square Size** `-s`: side length of the square in pixels. Default=100
- **Number of Squares** `-n`: the number of squares in one dimension of the simulation. Default=8
- **Color** `-c`: color of the squares. Default=(255, 255, 255)
- **Background Color** `-b`: color of the background. Default=(0, 0, 0)

Example: *blue sky with big clouds* 

`python display/grid.py -s 200 -n 4 -c 255,255,255 -b 0,0,255`
