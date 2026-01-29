import numpy as np
import cv2
from matplotlib import pyplot as plt

#Modified from https://parth3d.co.uk/3d-printing-aruco-tags-with-python-and-openscad

def getTagArray(num, tsz):
    # DICT_MXM_N
    # M is number of blocks 4 ... 7
    # N is number os possible tags 50, 100, 250 or 1000
    if tsz == 4:
        dict = cv2.aruco.DICT_4X4_50
    elif tsz == 5:
        dict = cv2.aruco.DICT_5X5_50
    elif tsz == 6:
        dict = cv2.aruco.DICT_6X6_50
    elif tsz == 7:
        dict = cv2.aruco.DICT_7X7_50
    else:
        return False
    tags = cv2.aruco.getPredefinedDictionary(dict)
    pixels = tsz + 2 * 1 # Tag plus border pixels
    tag = np.zeros((pixels, pixels, 1), dtype="uint8")
    # cv2.aruco.drawMarker(tags, num, pixels, tag, 1) # 1 is border pixels
    cv2.aruco.generateImageMarker(tags, num, pixels, tag, 1)
    return tag

def makeCubeScad(dx, dy, dz, ox=0, oy=0, oz=0):
    txt = ""
    if ox != 0 or oy != 0 or oz != 0:
        txt += "translate(["
        txt += str(ox) + ", " + str(oy) + ", " + str(oz)
        txt += "]) "
    txt += "cube(["
    txt += str(dx) + ", " + str(dy) + ", " + str(dz)
    txt += "], center=true);"
    return txt

def makeTagScad(tagdat, psz, bmm, sqmm, dmm,
                part="white", adj=0.01, upsidedown=False):

    tagsz = (psz * sqmm) + (2 * bmm)
    start = ((psz / 2) * sqmm) - (sqmm / 2)

    scad = ""

    # --------------------
    # WHITE = only white pixels
    # --------------------
    if part == "white":
        scad += "union()\n{\n"
        for y in range(psz):
            for x in range(psz):
                if tagdat[y+1][x+1] > 0:  # white pixels
                    if upsidedown:
                        px = start - (x * sqmm)
                    else:
                        px = -start + (x * sqmm)
                    py = start - (y * sqmm)

                    scad += "  " + makeCubeScad(
                        sqmm - adj,
                        sqmm - adj,
                        dmm,
                        px,
                        py,
                        dmm / 2
                    ) + "\n"
        scad += "}\n"

    # --------------------
    # BLACK = base minus white pixels
    # --------------------
    elif part == "black":
        scad += "difference()\n{\n"

        # Full black background
        scad += "  " + makeCubeScad(
            tagsz, tagsz, dmm, 0, 0, dmm / 2
        ) + "\n"

        # Subtract white pixels
        for y in range(psz):
            for x in range(psz):
                if tagdat[y+1][x+1] > 0:
                    if upsidedown:
                        px = start - (x * sqmm)
                    else:
                        px = -start + (x * sqmm)
                    py = start - (y * sqmm)

                    scad += "  " + makeCubeScad(
                        sqmm + adj,
                        sqmm + adj,
                        dmm * 3,
                        px,
                        py,
                        0
                    ) + "\n"

        scad += "}\n"

    return scad



tagnum = 11
tagsize = 4

bordermm = 10
squaremm = 10
depmm = 0.7

tag = getTagArray(tagnum, tagsize)

# White base
scad_white = makeTagScad(
    tag, tagsize, bordermm, squaremm, depmm,
    part="white", upsidedown=True
)

scad_black = makeTagScad(
    tag, tagsize, bordermm, squaremm, depmm,
    part="black", upsidedown=True
)

fname_white = f"tags/aruco_{tagnum}_{tagsize}x{tagsize}_white.scad"
fname_black = f"tags/aruco_{tagnum}_{tagsize}x{tagsize}_black.scad"

with open(fname_white, "w") as fp:
    fp.write(scad_white)

with open(fname_black, "w") as fp:
    fp.write(scad_black)

plt.imshow(tag, cmap='gray')
plt.axis('off')
plt.show()