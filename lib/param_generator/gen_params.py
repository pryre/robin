#!/usr/bin/env python

import yaml

path_params_yaml = "lib/param_generator/params.yaml"
path_param_gen_h = "lib/param_generator/param_gen.h"
path_param_gen_c = "lib/param_generator/param_gen.c"
path_params_md = "lib/param_generator/PARAMS.md"
path_params_pdf = "documents/params.pdf"

with open(path_params_yaml, 'r') as fstream:
	params = None

	# Load parameters
	try:
		params = yaml.load(fstream)
		print("Parameters Loaded: %i" % len(params))
	except yaml.YAMLError as e:
		print(e)
		exit()

	# Open files to insert generated data
	param_gen_h = open(path_param_gen_h, "w")
	param_gen_c = open(path_param_gen_c, "w")
	param_gen_md = open(path_params_md, "w")

	# Prepare file headers
	str_md = "# Parameter File Reference\n\nName | Type | Description | Default | Unit | Options | Reboot\n--- | --- | --- | ---:| --- | --- | ---\n"

	# Write file headers
	param_gen_md.write(str_md)

	print("Finished preparing file headers")

	try:
		# Iterate through every parameter
		for p in params.items():
			# Sanity Checks

			if p[1]["value"]["option"] == "scalar":
				param_type = p[1]["type"]
				param_val = p[1]["value"]["default"]
				param_val_type = type(param_val)
				raise_error_type_val = False

				# Default type
				if param_type == "float":
					if param_val_type != float:
						raise_error_type_val = True
				elif param_type == "int":
					if param_val_type != int:
						raise_error_type_val = True
				elif param_type == "uint":
					if (param_val_type != int) or (param_val < 0):
						raise_error_type_val = True

				if raise_error_type_val:
					raise ValueError("%s: Type (%s) and default value (%s) mismatch" % (p[0], param_type, str(param_val_type)))
			elif p[1]["value"]["option"] == "range":
				raise_error_type_val = False
				raise_error_type_range = False
				raise_error_min_max = False
				raise_error_out_of_range = False

				param_type = p[1]["type"]
				param_val = p[1]["value"]["default"]
				param_min = p[1]["value"]["min"]
				param_max = p[1]["value"]["max"]
				param_val_type = type(param_val)
				param_min_type = type(param_min)
				param_max_type = type(param_max)

				# Default type
				if param_type == "float":
					if param_val_type != float:
						raise_error_type_val = True
				elif param_type == "int":
					if param_val_type != int:
						raise_error_type_val = True
				elif param_type == "uint":
					if (param_val_type != int) or (param_val < 0):
						raise_error_type_val = True

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
					raise ValueError("%s: Type (%s) and default value (%s) mismatch" % (p[0], param_type, str(param_val_type)))
				if raise_error_type_range:
					raise ValueError("%s: Default value (%s) and range type (%s,%s) mismatch" % (p[0], str(param_val_type), str(param_min_type), str(param_max_type)))
				if raise_error_min_max:
					raise ValueError("%s: Invalid range: (%s > %s)" % (p[0], str(param_min), str(param_max)))
				if raise_error_out_of_range:
					raise ValueError("%s: Default not in set range: (%s <= %s <= %s)" % (p[0], str(param_min), str(param_val), str(param_max)))
			elif p[1]["value"]["option"] == "list":
				# TODO:
				# default is a uint
				# default is in size of list
				# all list options are of param type
				pass
			elif p[1]["value"]["option"] == "generated":
				# TODO:
				# function is a str
				pass
			else:
				raise ValueError("%s: Unsupported value option (%s)" % (p[0], p[1]["value"]["option"]))


			# Markdown generation
			str_md = p[1]["name"]
			str_md += " | "
			str_md += p[1]["type"]
			str_md += " | "
			str_md += p[1]["description"]
			str_md += " | "

			unit_str = ""

			if "unit" in p[1]["value"]:
				unit_str = p[1]["value"]["unit"]

			if p[1]["value"]["option"] == "scalar":
				str_md += str(p[1]["value"]["default"])
				str_md += " | "
				str_md = unit_str
				str_md += " | "
				str_md += "scalar"
				str_md += " | "
			elif p[1]["value"]["option"] == "range":
				str_md += str(p[1]["value"]["default"])
				str_md += " | "
				str_md = unit_str
				str_md += " | "
				str_md += "[min:%s, max:%s]" % (str(p[1]["value"]["min"]), str(p[1]["value"]["max"]))
				str_md += " | "
			elif p[1]["value"]["option"] == "list":
				str_md += str(p[1]["value"]["list"][p[1]["value"]["default"]])
				str_md += " | "
				str_md = unit_str
				str_md += " | "
				str_md += "%s" % str(p[1]["value"]["list"])
				str_md += " | "
			elif p[1]["value"]["option"] == "generated":
				str_md += " | | | "
			else:
				raise ValueError("%s: Unsupported value option (%s)" % (p[0], p[1]["value"]["option"]))

			str_md += str(p[1]["reboot"])

			str_md += "\n"

			param_gen_md.write(str_md)


		print("Finished parsing parameters")

		# Prepare file footers
		#str_md = ""

		# Write file footers
		#param_gen_md.write(str_md)
		print("Finished preparing file footers")
	except Exception as e:
		print("Error: %s" % e)

	# Close generated files
	param_gen_h.close()
	param_gen_c.close()
	param_gen_md.close()

	print("Finished parameter file generation")
