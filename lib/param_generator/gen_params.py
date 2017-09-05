#!/usr/bin/env python

import yaml
#path_params_pdf = "documents/params.pdf"

def read_params(param_file):
	with open(param_file, 'r') as fstream:
		# Load parameters
		try:
			params = yaml.load_all(fstream)
			params_ret = list(params)

			#for p in params_ret:
			#	print(p)
			#	print("\n")

			print("Parameters Loaded: %i" % len(params_ret))
			return params_ret
		except yaml.YAMLError as e:
			print(e)
			exit(1)

def val_is_int(v):
	check = True

	if type(v) != int:
		check = False

	return check

def val_is_uint(v):
	check = True

	if type(v) != int:
		check = False

	if v < 0:
		check = False

	return check


def val_is_float(v):
	check = True

	if type(v) != float:
		check = False

	return check

def val_check_type(val, expected_type):
	success = False

	if expected_type == "float":
		success = val_is_float(val)
	elif expected_type == "int":
		success = val_is_int(val)
	elif expected_type == "uint":
		success = val_is_uint(val)

	return success

def check_params(params):
	success = False

	try:
		id_list = []
		name_list = []

		for p in params:
			for param_id, details in p.items():
				#print(param_id)

				if not param_id in id_list:
					id_list.append(param_id)
				else:
					raise ValueError("%s: Parameter ID duplicate with param #%i" % (param_id, id_list.index(param_id)))

				if not details["name"] in name_list:
					name_list.append(details["name"])
				else:
					raise ValueError("%s: Parameter name duplicate with param %s" % (param_id, id_list[name_list.index(details["name"])]))

				if details["value"]["option"] == "scalar":
					param_type = details["type"]
					param_val = details["value"]["default"]
					param_val_type = type(param_val)
					raise_error_type_val = False

					# Default type
					raise_error_type_val = not val_check_type(param_val, param_type)

					if raise_error_type_val:
						raise ValueError("%s: Type (%s) and default value (%s) mismatch" % (param_id, param_type, str(param_val_type)))
				elif details["value"]["option"] == "range":
					raise_error_type_val = False
					raise_error_type_range = False
					raise_error_min_max = False
					raise_error_out_of_range = False

					param_type = details["type"]
					param_val = details["value"]["default"]
					param_min = details["value"]["min"]
					param_max = details["value"]["max"]
					param_val_type = type(param_val)
					param_min_type = type(param_min)
					param_max_type = type(param_max)

					# Default type
					raise_error_type_val = not val_check_type(param_val, param_type)

					# Range is same type as default
					if (param_val_type != param_min_type) or (param_val_type != param_max_type):
						raise_error_type_range = True

					# Min < Max
					if param_max < param_min:
						raise_error_min_max = True

					# Min < Default < Max
					if (param_val < param_min) or (param_val > param_max):
						raise_error_out_of_range = True

					if raise_error_type_val:
						raise ValueError("%s: Type (%s) and default value (%s) mismatch" % (param_id, param_type, str(param_val_type)))
					if raise_error_type_range:
						raise ValueError("%s: Default value (%s) and range type (%s,%s) mismatch" % (param_id, str(param_val_type), str(param_min_type), str(param_max_type)))
					if raise_error_min_max:
						raise ValueError("%s: Invalid range: (%s > %s)" % (param_id, str(param_min), str(param_max)))
					if raise_error_out_of_range:
						raise ValueError("%s: Default not in set range: (%s <= %s <= %s)" % (param_id, str(param_min), str(param_val), str(param_max)))
				elif details["value"]["option"] == "list":
					raise_error_type_val = False
					raise_error_type_list = False
					raise_error_out_of_range = False

					param_type = details["type"]
					param_val = details["value"]["default"]
					param_list = details["value"]["list"]
					param_val_type = type(param_val)

					# Default is a uint for index
					if (type(param_val) != int) or (param_val < 0):
							raise_error_type_val = True

					# Default is in size of list
					if param_val >= len(param_list):
						raise_error_out_of_range = True

					# All list options are of param type
					for l in param_list:
						raise_error_type_val = not val_check_type(l, param_type)

					if raise_error_type_val:
						raise ValueError("%s: Default value (%s) must be uint for index" % (param_id, str(param_val)))
					if raise_error_type_val:
						raise ValueError("%s: Default value (%s) exceeds number of items (%s)" % (param_id, str(param_val), str(len(param_list))))
					if raise_error_type_list:
						raise ValueError("%s: Type (%s) does not match all list values (%s)" % (param_id, str(param_type), str(param_list)))
				elif details["value"]["option"] == "generated":
					pass
				else:
					raise ValueError("%s: Unsupported value option (%s)" % (param_id, details["value"]["option"]))

		print("Paramters parsed OK!")
		success = True
	except ValueError as e:
		print("Error: %s" % e)

	return success

def gen_md(params, filepath):
	success = False

	# Open files to insert generated data
	param_gen_md = open(filepath, "w")

	try:
		# Prepare file headers
		str_md = "# Parameter File Reference\n\nparam_id | Type | Description | Default | Unit | Options | Reboot\n--- | --- | --- | ---:| --- | --- | ---\n"
		param_gen_md.write(str_md)

		# Markdown generation
		for p in params:
			for param_id, details in p.items():
				str_md = details["name"]
				str_md += " | "
				str_md += details["type"]
				str_md += " | "
				str_md += details["description"]
				str_md += " | "

				unit_str = ""

				if "unit" in details["value"]:
					unit_str = details["value"]["unit"]

				if details["value"]["option"] == "scalar":
					str_md += str(details["value"]["default"])
					str_md += " | "
					str_md += unit_str
					str_md += " | "
					str_md += "scalar"
					str_md += " | "
				elif details["value"]["option"] == "range":
					str_md += str(details["value"]["default"])
					str_md += " | "
					str_md += unit_str
					str_md += " | "
					str_md += "[min:%s, max:%s]" % (str(details["value"]["min"]), str(details["value"]["max"]))
					str_md += " | "
				elif details["value"]["option"] == "list":
					str_md += str(details["value"]["list"][details["value"]["default"]])
					str_md += " | "
					str_md += unit_str
					str_md += " | "
					str_md += "%s" % str(details["value"]["list"])
					str_md += " | "
				elif details["value"]["option"] == "generated":
					str_md += " | | | "
				else:
					raise ValueError("%s: Unsupported value option (%s)" % (param_id, details["value"]["option"]))

				str_md += str(details["reboot"])

				str_md += "\n"

				param_gen_md.write(str_md)

		# Prepare file footers
		#str_md = ""
		#param_gen_md.write(str_md)

		print("Finished generating Markdown: %s" % filepath)
		success = True
	except ValueError as e:
		print("Error: %s" % e)

	param_gen_md.close()

	return success


def gen_h(params, filepath):
	success = False

	param_gen_h = open(filepath, "w")

	try:
		# Prepare file headers
		str_h = "#pragma once\n\ntypedef enum {\n"
		param_gen_h.write(str_h)

		# Header generation
		for p in params:
			for param_id, details in p.items():
				str_h = "\t" + param_id + ",\n"
				param_gen_h.write(str_h)

		# Prepare file footers
		str_h = "\tPARAMS_COUNT\n} param_id_t;\n\nvoid params_init(void);\nvoid param_change_callback(param_id_t id);\n"
		param_gen_h.write(str_h)

		print("Finished generating H file: %s" % filepath)
		success = True
	except ValueError as e:
		print("Error: %s" % e)

	param_gen_h.close()

	return success

def gen_c(params, filepath):
	success = False

	param_gen_c = open(filepath, "w")

	try:
		# Prepare file headers
		str_c = "#include \"params.h\"\n"
		str_c += "#include \"param_generator/param_gen.h\"\n"
		str_c += "#include \"mavlink_system.h\"\n"
		str_c += "#include \"mavlink_transmit.h\"\n"
		str_c += "#include \"controller.h\"\n"
		str_c += "#include \"pid_controller.h\"\n"
		str_c += "#include \"fix16.h\"\n\n"
		str_c += "void set_param_defaults(void) {\n"
		param_gen_c.write(str_c)

		# C generation
		for p in params:
			for param_id, details in p.items():
				start_str = "init_param_"

				if details["type"] == "uint":
					start_str += "uint" + "(" + param_id + ", \"" + details["name"] + "\", "
				elif details["type"] == "int":
					start_str += "int" + "(" + param_id + ", \"" + details["name"] + "\", "
				elif details["type"] == "float":
					start_str += "fix16" + "(" + param_id + ", \"" + details["name"] + "\", "
				else:
					raise ValueError("%s: Unsupported type option (%s)" % (param_id, details["type"]))

				val_str = ""

				if details["value"]["option"] == "scalar":
					if details["type"] == "float":
						val_str = "fix16_from_float(" + str(details["value"]["default"]) + "f)"
					else:
						val_str = str(details["value"]["default"])
				elif details["value"]["option"] == "range":
					if details["type"] == "float":
						val_str = "fix16_from_float(" + str(details["value"]["default"]) + "f)"
					else:
						val_str = str(details["value"]["default"])
				elif details["value"]["option"] == "generated":
					val_str = details["value"]["function"]
				elif details["value"]["option"] == "list":
					val = details["value"]["list"][details["value"]["default"]]

					if details["type"] == "float":
						val_str = "fix16_from_float(" + str(val) + "f)"
					else:
						val_str = str(val)
				else:
					raise ValueError("%s: Unsupported value option (%s)" % (param_id, details["value"]["option"]))

				str_c = "\t" + start_str + val_str + ");\n"
				param_gen_c.write(str_c)

		str_c = "}\n\n"
		param_gen_c.write(str_c)

		str_c = "void param_change_callback(param_id_t id) {\n\tswitch(id) {\n"
		param_gen_c.write(str_c)

		for p in params:
			for param_id, details in p.items():

				if ("function" in details["value"]) and not (details["value"]["option"] == "generated"):
					str_c = "\t\tcase " + param_id + ":\n"
					str_c += "\t\t\t" + details["value"]["function"] + "\n"
					str_c += "\t\t\tbreak;\n"
					param_gen_c.write(str_c)


		str_c = "\t\tdefault:\n"
		str_c += "\t\t\t//No action needed for this param ID\n"
		str_c += "\t\t\tbreak;\n"
		str_c += "\t}\n\n"
		str_c += "\tmavlink_message_t msg_out;\n"
		str_c += "\tmavlink_prepare_param_value( &msg_out, id );\n"
		str_c += "\tlpq_queue_broadcast_msg( &msg_out );\n"
		str_c += "}\n"
		param_gen_c.write(str_c)

		print("Finished generating C file: %s" % filepath)
		success = True
	except ValueError as e:
		print("Error: %s" % e)

	param_gen_c.close()

	return success

def main():

	path_params_yaml = "lib/param_generator/params.yaml"
	path_params_md = "lib/param_generator/PARAMS.md"
	path_params_h = "lib/param_generator/param_gen.h"
	path_params_c = "lib/param_generator/param_gen.c"

	params = read_params(path_params_yaml)

	#TODO: Still need to check for unique param_ids
	if not check_params(params):
		exit(1)

	if not gen_md(params, path_params_md):
		exit(1)

	if not gen_h(params, path_params_h):
		exit(1)

	if not gen_c(params, path_params_c):
		exit(1)

	print("Finished parameter file generation")


if __name__ == "__main__":
    main()
