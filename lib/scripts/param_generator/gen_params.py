#!/usr/bin/env python

import os
import sys
import yaml
import glob
#path_params_pdf = "documents/params.pdf"

def read_params(param_file):
	with open(param_file, 'r') as fstream:
		# Load parameters
		try:
			params = yaml.safe_load(fstream)

			#for p in params_ret:
			#	print(p)
			#	print("\n")

			print("\t%s: \t%i" % ( os.path.basename(param_file).split('.')[0], len(params) ) )
			return params
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

		for key, value in params.items():
			param_id = key
			details = value

			# XXX: This is now checked on-load of dictionary
			# if not param_id in id_list:
			# 	id_list.append(param_id)
			# else:
			# 	raise ValueError("%s: Parameter ID duplicate with param #%i" % (param_id, id_list.index(param_id)))

			if not details["name"] in name_list:
				name_list.append(details["name"])
			else:
				raise ValueError("%s: Parameter name duplicate with param %s" % (param_id, id_list[name_list.index(details["name"])]))

			#XXX: Length 16 is from MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN
			if len(details["name"]) > 16:
				raise ValueError("%s: Parameter name too long (%d>16)" % (param_id, len(details["name"])))

			if details["value"]["option"] == "bool":
				param_type = details["type"]
				param_val = details["value"]["default"]
				param_val_type = type(param_val)
				raise_error_type_val = False
				raise_error_bad_val = False

				# Default type
				raise_error_type_val = not val_check_type(param_val, param_type)

				if not ( (param_val == 0) or (param_val == 1) ):
					raise_error_bad_val = True

				if raise_error_type_val:
					raise ValueError("%s: Type (%s) and default value (%s) mismatch" % (param_id, param_type, str(param_val_type)))
				if raise_error_bad_val:
					raise ValueError("%s: Value (%s) must be either 0 or 1 for option 'bool'" % (param_id, str(param_val)))
			elif details["value"]["option"] == "scalar":
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
				raise ValueError("%s: Unsupported value option (%s) for parameter checking" % (param_id, details["value"]["option"]))

		print("Paramters parsed OK!")
		success = True
	except ValueError as e:
		print("Error: %s" % e)

	return success

def gen_md(params_g, param_groups, filepath):
	success = False

	# Open files to insert generated data
	param_gen_md = open(filepath, "w")

	try:
		# Prepare file headers
		str_md = "# Parameter File Reference\n"
		param_gen_md.write(str_md)
		str_md = "[Back to index](/documents/README.md).\n\n"
		param_gen_md.write(str_md)

		for i in range(len(param_groups)):
			param_gen_md.write("- [%s](/documents/autogen/PARAM_LIST.md#%s)\n\n" % (param_groups[i],param_groups[i].replace(' ', '-')))

		str_md_tab = "Name | Type | Description | Default | Unit | Options | Reboot\n--- | --- | --- | ---:| --- | --- | ---\n"

		for i in range(len(param_groups)):
			str_md = ("## %s\n\n" % param_groups[i]) + str_md_tab
			param_gen_md.write(str_md)

			# Markdown generation
			for key, value in params_g[i].items():
				param_id = key
				details = value

				str_md = details["name"]
				str_md += " | "
				str_md += details["type"]
				str_md += " | "
				str_md += details["description"]
				str_md += " | "

				unit_str = ""

				if "unit" in details["value"]:
					unit_str = details["value"]["unit"]

				if details["value"]["option"] == "bool":
					str_md += str(details["value"]["default"])
					str_md += " | "
					str_md += "0 / 1"
					str_md += " | "
					str_md += "boolean"
					str_md += " | "
				elif details["value"]["option"] == "scalar":
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
					raise ValueError("%s: Unsupported value option (%s) for Markdown generation" % (param_id, details["value"]["option"]))

				str_md += str(details["reboot"])
				str_md += "\n"
				param_gen_md.write(str_md)

			str_md = "\n"
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
		str_h = """#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_system.h"
#include <mavlink/common/mavlink.h>

typedef enum {
"""
		param_gen_h.write(str_h)

		# Header generation
		for key, value in params.items():
			param_id = key
			details = value

			str_h = "\t" + param_id + ",\n"
			param_gen_h.write(str_h)

		# Prepare file footers
		str_h = """\tPARAMS_COUNT
} param_id_t;

extern const char _param_names[PARAMS_COUNT][MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];
extern const MAV_PARAM_TYPE _param_types[PARAMS_COUNT];

void params_init( void );
void set_param_defaults( void );
void param_change_callback(param_id_t id);

#ifdef __cplusplus
}
#endif
"""
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
		str_c = """#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>

#include "params.h"
#include "param_gen.h"
#include "mavlink_system.h"
#include "mavlink_transmit.h"
#include "controllers/controller_att_pid.h"
#include "controllers/control_lib_pid.h"
#include "sensors.h"
#include "profiler.h"
#include "fix16.h"

void set_param_defaults(void) {"""
		param_gen_c.write(str_c)

		# C generation
		for key, value in params.items():
			param_id = key
			details = value

			start_str = "init_param_"

			if details["type"] == "uint":
				start_str += "uint" + "(" + param_id + ", "
			elif details["type"] == "int":
				start_str += "int" + "(" + param_id + ", "
			elif details["type"] == "float":
				start_str += "fix16" + "(" + param_id + ", "
			else:
				raise ValueError("%s: Unsupported type option (%s)" % (param_id, details["type"]))

			val_str = ""

			if details["value"]["option"] == "bool":
				val_str = str(details["value"]["default"])
			elif details["value"]["option"] == "scalar":
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
				raise ValueError("%s: Unsupported value option (%s) for C generation" % (param_id, details["value"]["option"]))

			str_c = "\t" + start_str + val_str + ");\n"
			param_gen_c.write(str_c)

		str_c = "}\n\n"
		param_gen_c.write(str_c)

		str_c = "const char _param_names[PARAMS_COUNT][MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = {\n"
		param_gen_c.write(str_c)

		for key, value in params.items():
			param_id = key
			details = value

			str_c = "\t\"" + details["name"] + "\",\n"
			param_gen_c.write(str_c)

		str_c = "};\n\n"
		param_gen_c.write(str_c)

		str_c = "const MAV_PARAM_TYPE _param_types[PARAMS_COUNT] = {\n"
		param_gen_c.write(str_c)

		for key, value in params.items():
			param_id = key
			details = value

			mavlink_type = ''
			if details["type"] == "uint":
				mavlink_type = "MAVLINK_TYPE_UINT32_T"
			elif details["type"] == "int":
				mavlink_type = "MAVLINK_TYPE_INT32_T"
			elif details["type"] == "float":
				mavlink_type = "MAVLINK_TYPE_FLOAT"
			else:
				raise ValueError("%s: Unsupported type option (%s)" % (param_id, details["type"]))

			str_c = "\t(MAV_PARAM_TYPE)" + mavlink_type + ",\n"
			param_gen_c.write(str_c)

		str_c = "};\n\n"
		param_gen_c.write(str_c)

		str_c = "void param_change_callback(param_id_t id) {\n\tswitch(id) {\n"
		param_gen_c.write(str_c)

		for key, value in params.items():
			param_id = key
			details = value

			if ("function" in details["value"]) and not (details["value"]["option"] == "generated"):
				str_c = "\t\tcase " + param_id + ":\n"
				str_c += "\t\t\t" + details["value"]["function"] + "\n"
				str_c += "\t\t\tbreak;\n"
				param_gen_c.write(str_c)


		str_c = """		default:
			//No action needed for this param ID
			break;
	}

	lpq_queue_param_broadcast(id);
}

#ifdef __cplusplus
}
#endif
"""
		param_gen_c.write(str_c)

		print("Finished generating C file: %s" % filepath)
		success = True
	except ValueError as e:
		print("Error: %s" % e)

	param_gen_c.close()

	return success

def main():
	proj_dir_in = str(sys.argv[1])
	proj_dir_out = str(sys.argv[2])

	print('Loading parameter definitions from: %s' % os.path.dirname(proj_dir_in))
	path_params_yaml = sorted(glob.glob( os.path.dirname(proj_dir_in) + "/*.yaml"))

	param_groups = []
	for ppy in path_params_yaml:
		param_groups.append(os.path.basename(ppy).split('.')[0])

	if not os.path.exists(proj_dir_out):
		os.mkdir(proj_dir_out)

	path_params_md = os.path.dirname(proj_dir_out) + "/PARAM_LIST.md"
	path_params_h = os.path.dirname(proj_dir_out) + "/param_gen.h"
	path_params_c = os.path.dirname(proj_dir_out) + "/param_gen.c"

	do_gen = False

	try:
		mod_time_yaml = 0
		for pf in path_params_yaml:
			mod_time_yaml_pf = os.stat(pf).st_mtime

			if mod_time_yaml == 0:
				mod_time_yaml = mod_time_yaml_pf
			else:
				if mod_time_yaml < mod_time_yaml_pf:
					mod_time_yaml = mod_time_yaml_pf

		if mod_time_yaml > os.stat(path_params_md).st_mtime:
			do_gen = True

		if mod_time_yaml > os.stat(path_params_h).st_mtime:
			do_gen = True

		if mod_time_yaml > os.stat(path_params_c).st_mtime:
			do_gen = True

	except:
		do_gen = True

	if do_gen:
		params = {} #Raw parameter list
		params_g = [] # Grouped parameter list
		# param_count = 0

		for pf in path_params_yaml:
			pf_list = read_params(pf)
			pf_cross = [key for key in params.keys() & pf_list.keys()]

			if len(pf_cross) > 0:
				print('Error: The following keys have been redefined (%s):' % pf)
				print(pf_cross)
				exit(1)
			else:
				# param_count += len(pf_list)
				params.update(pf_list)
				params_g.append(pf_list)

		if len(param_groups) == len(params_g):
			print("Total parameters loaded: %i" % len(params))
		else:
			print("Error: Parameters loaded do not match expected number of groups")
			exit(1)

		if not check_params(params):
			exit(1)

		if not gen_md(params_g, param_groups, path_params_md):
			exit(1)

		if not gen_h(params, path_params_h):
			exit(1)

		if not gen_c(params, path_params_c):
			exit(1)

		print("Finished parameter file generation")
	else:
		print("Parameter files up to date")

if __name__ == "__main__":
    main()
