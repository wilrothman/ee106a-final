Run command:
`python3 display/game.py [flags]`, where the desired flags are enumerated in no particular order.


- Shape: `-s` for square or `-c` for circle. *Default: square.*
- Square side length or radius: `-r [desired size]` where the number is a number or range of numbers. `Default: 10% of average of display dimension.`
- Display Dimensions: `-display [display dimensions]` or `-d [display dimensions]`. *Default: `(800, 600)`*. 
- Background Color: `-background [color]` or `-b [color]` color of the background. *Default: `(0, 0, 0)`.*
- Shape Color: `-color [color]`. *Default `(255, 255, 255)`.*

Examples:

`>>> python3 display/game.py -s -r 70 -display (800, 600) -background (0, 0, 0) -color (255, 255, 255) # default settings`

`>>> python3 display/game.py -c -r (50, 100) -d (800, 200) -b (255, 255, 255) -c (0, 0, 0)`
