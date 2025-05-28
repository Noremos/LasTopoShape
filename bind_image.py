
import numpy as np

import laspy
from PIL import Image

def geobind_image_with_las(image_path, las_file_path, output_image_path):
	with laspy.open(las_file_path) as las:
		x_min = las.header.mins[0]
		y_min = las.header.mins[1]
		x_max = las.header.maxs[0]
		y_max = las.header.maxs[1]

		# image the image
		image = Image.open(image_path)
		if image is None:
			raise FileNotFoundError(f"Image file {image_path} could not be opened.")

		points = [
			{"pixelX": 0, 			"pixelY": image.height,		"mapX": x_min, "mapY": y_min},
			{"pixelX": 0,			"pixelY": 0, 				"mapX": x_min, "mapY": y_max},
			{"pixelX": image.width, "pixelY": image.height,		"mapX": x_max, "mapY": y_min},
			{"pixelX": image.width, "pixelY": 0,				"mapX": x_max, "mapY": y_max}
		]

		with open(output_image_path, "w") as f:
			for pt in points:
				f.write(f"{pt['mapX']}\t{pt['mapY']}\t{pt['pixelX']}\t{pt['pixelY']}\n")


if __name__ == '__main__':
	geobind_image_with_las('big.jpg','Ground_2020_Участок 1.las','image.points')