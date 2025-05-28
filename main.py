import numpy as np

import laspy
import shapefile
import time
import os

import ImageTopoDec as bc

class LasReaderSetup:
	"""
	Класс для настройки параметров чтения и обработки LAS файлa.
	"""
	def __init__(self):
		self.aproximateRectSize = 10
		self.chunkSize = 1500
		self.chunkOversize = 100
		self.useMask = False
		self.procType = bc.ProcType.Radius

	def get_full_tile_size(self):
		"""
		Возвращает полный размер плитки с учетом наложения чанка.
		"""
		return self.chunkSize + self.chunkOversize

	def get_full_chunk_size_to_read(self):
		"""
		Возвращает полный размер чанка для чтения с учетом наложения.
		"""
		return (self.chunkSize + self.chunkOversize) * self.aproximateRectSize

	def get_chunk_size_to_read(self):
		"""
		Возвращает размер чанка без учета наложения.
		"""
		return self.chunkSize * self.aproximateRectSize

	pass # end of class


class TileProjector:
	"""
	Класс для преобразования координат матрицы в глобальные координаты.
	"""
	def __init__(self, factor, offsetX, offsetY, locMutator):
		"""
		Инициализирует параметры проекции.

		factor -- масштабы по осям (tuple)
		offsetX, offsetY -- смещения по осям
		locMutator -- модификатор координат
		"""
		self.scales = factor
		self.offsetX = offsetX
		self.offsetY = offsetY
		self.locMutator = locMutator

	def tileToGlobal(self, locX, locY):
		"""
		Преобразует координаты плитки в глобальные координаты.

		locX, locY -- координаты на плитке

		Возвращает кортеж (x, y) в глобальной системе координат.
		"""
		return (
			(self.offsetX + locX * self.locMutator) * self.scales[0],
			(self.offsetY + locY * self.locMutator) * self.scales[1]
		)

	pass  # end of class


class LasChunkReader:
	"""
	Класс для чтения чанков из LAS файла.
	"""
	def __init__(self, lasData: laspy.LasReader, setup: LasReaderSetup):
		"""
		Инициализирует чтение LAS файла, конвертируя точки в словарь.

		lasData -- объект чтения LAS файла
		setup   -- настройки чтения
		"""
		self.setup = setup
		self.las = lasData
		print(f"Файл содержит {self.las.header.point_count} точек")
		print("Чтение файла...", end=' ', flush=True)

		las_points = self.las.read().points
		self.scales = las_points.scales

		self.x = int(self.las.header.mins[0] / self.scales[0])
		self.y = int(self.las.header.mins[1] / self.scales[1])
		self.endX = int(self.las.header.maxs[0] / self.scales[0])
		self.endY = int(self.las.header.maxs[1] / self.scales[1])
		print("Готово")

		print("Конвертация точек...", end=' ', flush=True)
		start = time.time()
		self.points = dict(zip(zip(las_points.array['X'], las_points.array['Y']), las_points.array['Z']))
		end = time.time()
		print(f"Готово ({end - start} секунд)")
		print(f"Границы данных: x: {self.x * self.scales[0]} y: {self.y * self.scales[1]} endX: {self.endX * self.scales[0]} endY: {self.endY * self.scales[1]}")

	def get_read_range_x(self):
		"""
		Возвращает диапазон координат X для чтения чанков.
		"""
		return range(self.x, self.endX, self.setup.get_chunk_size_to_read())

	def get_read_range_y(self):
		"""
		Возвращает диапазон координат Y для чтения чанков.
		"""
		return range(self.y, self.endY, self.setup.get_chunk_size_to_read())

	def get_chunk_end_x(self, chunk_start_x):
		"""
		Вычисляет конечное значение координаты X для чанка.

		chunk_start_x -- начальное значение координаты X чанка

		Возвращает минимальное значение между границей чанка и концом диапазона.
		"""
		return min(chunk_start_x + self.setup.get_full_chunk_size_to_read(), self.endX)

	def get_chunk_end_y(self, chunk_start_y):
		"""
		Вычисляет конечное значение координаты Y для чанка.

		chunk_start_y -- начальное значение координаты Y чанка

		Возвращает минимальное значение между границей чанка и концом диапазона.
		"""
		return min(chunk_start_y + self.setup.get_full_chunk_size_to_read(), self.endY)


	def get_value(self, x, end_x, y, end_y, aproximateRectSize):
		"""
		Считает сумму значений и количество точек в заданном прямоугольнике.

		Читаем точки в квадрате aproximateRectSize на aproximateRectSize
		возвращаем их среднее значение

		x, y           -- начальные координаты прямоугольника
		end_x, end_y   -- конечные координаты прямоугольника
		aproximateRectSize -- размер прямоугольника для аппроксимации

		Возвращает кортеж (сумма, количество).
		"""
		sum = 0.0
		count = 0

		max_x = min(x + aproximateRectSize, end_x)
		max_y = min(y + aproximateRectSize, end_y)
		getPoint = self.points.get

		for ax in range(x, max_x):
			for ay in range(y, max_y):
				value = getPoint((ax, ay), None)
				if value is not None:
					sum += value
					count += 1

		return sum, count

	def print_chunk_grid(self, read_start_x, read_start_y):
		"""
		Выводит ASCII-грид текущей сетки чанка для визуализации обработки.

		read_start_x, read_start_y -- начальные координаты чтения чанка.
		"""
		num_chunks_x = get_tile_size(self.endX - self.x, self.setup.get_chunk_size_to_read())
		num_chunks_y = get_tile_size(self.endY - self.y, self.setup.get_chunk_size_to_read())

		chunk_idx_x = (read_start_x - self.x) // self.setup.get_chunk_size_to_read()
		chunk_idx_y = (read_start_y - self.y) // self.setup.get_chunk_size_to_read()

		chunkSymbol = 'X'  # Чанк прочитан
		grid = []
		for x in range(num_chunks_x):
			row = ''
			for y in range(num_chunks_y):
				if x == chunk_idx_x and y == chunk_idx_y:
					row += '*'
					chunkSymbol = '.'  # Чанк не прочитан
				else:
					row += chunkSymbol
			grid.append(row)

		for row in grid:
			print(row)

	def read_chunk(self, read_start_x, read_start_y, aproximateRectSize) -> np.ndarray | np.ndarray:
		"""
		Считывает чанк из LAS файла и формирует массив высот и маску.

		read_start_x, read_start_y -- начальные координаты чтения чанка.
		aproximateRectSize -- размер аппроксимации для снижения разрешения.

		Возвращает кортеж (chunk, mask) - массив высот и булевую маску.
		"""
		print("Чтение чанка...")
		self.print_chunk_grid(read_start_x, read_start_y)

		defaultValue = 9999 if self.setup.procType == bc.ProcType.f0t255 else -9999

		read_end_x = self.get_chunk_end_x(read_start_x)
		read_end_y = self.get_chunk_end_y(read_start_y)

		# Т.к. в las точки неравномерны и встречаются с промежутками,
		# апроксимируем квадрат aproximateRectSize на aproximateRectSize точек в 1 пиксель
		tile_width = get_tile_size(read_end_x - read_start_x, aproximateRectSize)
		tile_height = get_tile_size(read_end_y - read_start_y, aproximateRectSize)

		# Создаём выходной массив и маску
		chunk = np.zeros((tile_height, tile_width), dtype=np.float32)
		mask = np.zeros((tile_height, tile_width), dtype=np.int8)

		# Читаем точки в чанке
		out_x = 0
		for x in range(read_start_x, read_end_x, aproximateRectSize):
			out_y = 0
			for y in range(read_start_y, read_end_y, aproximateRectSize):
				sum, count = self.get_value(x, read_end_x, y, read_end_y, aproximateRectSize)

				if count > 0:
					chunk[out_y][out_x] = sum / count
					mask[out_y, out_x] = 1
				else:
					chunk[out_y][out_x] = defaultValue
					mask[out_y, out_x] = 0

				out_y += 1
				pass

			out_x += 1
			pass

		return chunk, mask

	pass # end of class


def get_tile_size(width, aproximateRectSize):
	"""
	Вычисляет размер плитки с учетом размера аппроксимации.

	width -- ширина участка
	aproximateRectSize -- размер аппроксимации

	Возвращает количество плиток, необходимых для покрытия ширины.
	"""
	a = width // aproximateRectSize
	b = width % aproximateRectSize
	return a + (1 if b != 0 else 0)


class ShapeFileWriter:
	"""
	Класс для записи данных в shapefile.
	"""
	def __init__(self, file_name):
		"""
		Инициализирует writer для shapefile с заданными полями.

		file_name -- имя файла для сохранения shapefile.
		"""
		self.file_name = file_name
		self.writer = shapefile.Writer(file_name, shapeType=shapefile.POLYGON)
		self.writer.field('Id', 'N')
		self.writer.field('ParentId', 'N')
		self.writer.field('NumPoints', 'N')
		self.writer.field('Start', 'N', decimal=5)
		self.writer.field('End', 'N', decimal=5)
		self.writer.field('Depth', 'N')

	def find_contour_order(self, polygon: bc.Barline, projection: TileProjector):
		"""
		Определяет порядок обхода контура с использованием алгоритма Moore-Neighbor tracing.

		polygon   -- объект полигона
		projection -- объект преобразования координат

		Возвращает список точек контура в глобальных координатах.
		"""
		points = set()
		leftPoint = (polygon.getMatrixValue(0).x, polygon.getMatrixValue(0).y)

		for i in range(polygon.getMatrixSize()):
			value = polygon.getMatrixValue(i)
			points.add((value.x, value.y))
			points.add((value.x + 1, value.y))
			points.add((value.x, value.y + 1))
			points.add((value.x + 1, value.y + 1))
			if value.x < leftPoint[0]:
				leftPoint = (value.x, value.y)

		if len(points) < 3:
			return []

		poss = [(-1, 0), (-1, -1), (0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1)]
		contour = []
		start_point = leftPoint
		cur = start_point

		max_iter = 10000  # предотвращение бесконечного цикла

		def exists(pt):
			return pt in points

		direction = 4

		for _ in range(max_iter):
			start = (direction + 6) % 8
			end = start + 7

			found = False
			for i in range(start, end):
				idx = (start + i) % 8
				offset = poss[idx]
				new_point = (cur[0] + offset[0], cur[1] + offset[1])
				if exists(new_point):
					newDirection = idx % 8
					if direction != newDirection:
						contour.append(projection.tileToGlobal(cur[0], cur[1]))

					cur = new_point
					direction = newDirection
					found = True
					break
			assert found, "No neighbor found, infinite loop detected"
			if not found:
				break
			if cur == start_point:
				if not contour or contour[0] != start_point:
					contour.append(projection.tileToGlobal(cur[0], cur[1]))
				break

		return contour

	def write_polygon_record(self, polygon: bc.Barline, xm: bc.PointMutator, ym: bc.PointMutator):
		"""
		Записывает полигональную запись в shapefile, если контур корректен.

		polygon -- объект полигона
		xm, ym  -- объекты с настройками преобразования координат
		"""
		if polygon.getMatrixSize() < 15:
			return

		poly = bc.find_contour(polygon, xm, ym, True)
		if len(poly) < 3:
			return

		self.writer.poly([poly])
		self.writer.record(
			Id=polygon.id(),
			ParentId=polygon.parentId(),
			NumPoints=polygon.getMatrixSize(),
			Start=polygon.start().getAvgFloat(),
			End=polygon.end().getAvgFloat(),
			Depth=polygon.depth()
		)

	def close(self):
		"""
		Закрывает writer и сохраняет shapefile.
		"""
		self.writer.close()


def process_las_file(las_file_path, output, convertor: LasReaderSetup):
	"""
	Обрабатывает LAS файл, разбивая его на чанки, и создает shapefile.

	las_file_path -- путь к LAS файлу
	output        -- путь для сохранения выходного shapefile
	convertor     -- настройки для обработки LAS файла
	"""
	shp = ShapeFileWriter(output)
	with laspy.open(las_file_path) as lasData:
		las = LasChunkReader(lasData, convertor)

		# Обрабатываем las как карту высот
		# Т.к. для обработки всех точек не хватит оперативной памяти,
		# то разбиваем на чанки
		for read_start_x in las.get_read_range_x():
			for read_start_y in las.get_read_range_y():

				# Читаем чанк
				chunk, mask = las.read_chunk(read_start_x, read_start_y, convertor.aproximateRectSize)
				xm = bc.PointMutator()
				ym = bc.PointMutator()
				ym.local = xm.local = convertor.aproximateRectSize
				xm.offset = read_start_x
				ym.offset = read_start_y
				print("Обработка чанка...")
				print("X: {} Y: {}".format(xm.offset, ym.offset))
				bcs = bc.barstruct()
				bcs.coltype = bc.ColorType.native
				bcs.proctype = convertor.procType
				bcs.createBinaryMasks = True
				bcs.createGraph = True
				bcs.maskValueId = 1

				if convertor.useMask:
					item = bc.createByMask(chunk, bcs, mask)
				else:
					item = bc.create(chunk, bcs)

				lines = item.getBarcodeLines()
				for line in lines:
					shp.write_polygon_record(line, xm, ym)

				print("------Переход к следующему чанку-----")

	shp.close()
	pass



def process_single(las_file_path, output, procType):
	"""
	Обрабатывает один LAS файл с заданным типом обработки.

	las_file_path -- путь к LAS файлу
	output        -- путь для сохранения выходного shapefile
	procType      -- тип обработки
	"""
	preferences = LasReaderSetup()

	# В las точки неравномерны, между x1 и x10 может быть 3-5 точек,
	# поэтому нужно уменьшить размерность
	preferences.aproximateRectSize = 10

	# Целиком обработка файла слишком затратна из-а большого размера
	# Для выполнения обработки на обычном компьютере необходимо разбить на чанки
	# Размер указывается в пикселях после уменьшения размерности
	preferences.chunkSize = 1500

	# Так как важные объекты могут быть на границе чанка, нужно брать с захлёстом
	# В идеале размер заълёста - максимальный размер объекта
	preferences.chunkOversize = 100


	preferences.procType = procType


	preferences.useMask = False

	process_las_file(las_file_path, output, preferences)
	pass


def make_output_dir():
	"""
	Создает директорию для shapefile, если она не существует.
	"""
	if not os.path.exists("shapefiles"):
		os.makedirs("shapefiles")


def process_multiple_files(lasFiles, processTypes=[bc.ProcType.Radius]):
	"""
	Обрабатывает несколько LAS файлов для каждого типа обработки.

	lasFiles     -- список путей к LAS файлам
	processTypes -- список типов обработки
	"""
	make_output_dir()
	for proc in processTypes:
		for las in lasFiles:
			name = os.path.basename(las)
			name = name.replace('.las', '.shp')
			outputName = os.path.join("shapefiles", str(proc).replace('.', '_'), name)
			process_single(las, outputName, proc)


if __name__ == "__main__":
	lasFiles = [
		'./Безклассификации/2020_1 без классификации_ч1.las',
		'./Безклассификации/2020_1 без классификации_ч2.las',
		'./Ground_2020_Участок 1.las'
	]
	allProcessTypes = [bc.ProcType.f0t255, bc.ProcType.f255t0, bc.ProcType.Radius]
	process_multiple_files(lasFiles, allProcessTypes)
