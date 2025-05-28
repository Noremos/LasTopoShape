import argparse
import sys
import main as topo

def parse_arguments():
	parser = argparse.ArgumentParser(
		description="Обработка LAS файла с применением настроек для чанков и обработки."
	)
	parser.add_argument("input_file", help="Имя входного LAS файла")
	parser.add_argument("output_file", help="Имя выходного shape файла")
	parser.add_argument("--chunk_size", type=int, default=1000, help="Размер чанка")
	parser.add_argument("--chunk_oversize", type=int, default=100, help="Размер наложения чанка")
	parser.add_argument("--aproximate_rect_size", type=int, default=10, help="Окно усреднения")
	parser.add_argument("--proc_type", type=int, default=1, help="Тип обработки (0 - from min, 1 - from max, 2 - radius)")
	args = parser.parse_args()

	return args

print(int(topo.bc.ProcType.f255t0))

if __name__ == "__main__":
	try:
		args = parse_arguments()
	except Exception as e:
		print(f"Ошибка при разборе аргументов командной строки: {e}", file=sys.stderr)
		sys.exit(1)

	setup = topo.LasReaderSetup()
	try:
		setup.chunkSize = args.chunk_size
		setup.chunkOversize = args.chunk_oversize
		setup.aproximateRectSize = args.aproximate_rect_size
		setup.procType = topo.bc.ProcType(args.proc_type)
	except Exception as e:
		print(f"Ошибка при инициализации настроек: {e}", file=sys.stderr)
		sys.exit(1)

	try:
		topo.process_las_file(args.input_file, args.output_file, setup)
	except Exception as e:
		print(f"Ошибка при обработке LAS файла: {e}", file=sys.stderr)
		sys.exit(1)