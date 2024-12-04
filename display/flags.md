Run command:
`python3 display/game.py [flags]`, where the desired flags are enumerated in no particular order.


- Shape: `-s [square or circle]`. *Default: square.*
- Square side length or radius: `-r [desired size]` where the number is a number or range of numbers. `Default: 10% of average of display dimension.`
- Display Dimensions: `-d [display dimensions]`. *Default: `(800, 600)`*. 
- Background Color: `-b [color]` color of the background. *Default: `(0, 0, 0)`.*
- Shape Color: `-c [color]`. *Default `(255, 255, 255)`.*

Examples:

`>>> python3 display/game.py -s square -r 70 -d 800,600 -b 0,0,0 -c 255,255,255 # default settings`

`>>> python3 display/game.py -s circle -r 50,100 -d 800,200 -b 255,255,255 -c 0,0,0`
