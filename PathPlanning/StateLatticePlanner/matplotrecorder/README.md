# matplotrecorder
A simple Python module for recording matplotlib animation

It can generate a matplotlib animation movie (mp4, gif, etc.)

This tool use "convert" command of ImageMagick.

# Sample gif

![matplotrecorder/animation.gif at master · AtsushiSakai/matplotrecorder](https://github.com/AtsushiSakai/matplotrecorder/blob/master/animation.gif)

# Requrements

- [ImageMagic](https://www.imagemagick.org/script/index.php)


# How to use

Call save_frame() at each animation iteration,

And then, call savemovie() for movie generation.

A sample code:


      import matplotrecorder

      print("A sample recording start")
      import math

      time = range(50)

      x1 = [math.cos(t / 10.0) for t in time]
      y1 = [math.sin(t / 10.0) for t in time]
      x2 = [math.cos(t / 10.0) + 2 for t in time]
      y2 = [math.sin(t / 10.0) + 2 for t in time]

      for ix1, iy1, ix2, iy2 in zip(x1, y1, x2, y2):
          plt.plot(ix1, iy1, "xr")
          plt.plot(ix2, iy2, "xb")
          plt.axis("equal")
          plt.pause(0.1)

          matplotrecorder.save_frame()  # save each frame

      # generate movie
      matplotrecorder.save_movie("animation.mp4", 0.1)
      #  matplotrecorder.save_movie("animation.gif", 0.1) #gif is ok.



# License 

MIT

# Author

Atsushi Sakai ([@Atsushi_twi](https://twitter.com/Atsushi_twi))

