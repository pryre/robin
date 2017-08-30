#!/usr/bin/env python

import yaml

with open("lib/param_generator/params.yaml", 'r') as fstream:
	params = None

	# Load parameters
	try:
		params = yaml.load(fstream)
		print("Parameters Loaded: %i" % len(params))
	except yaml.YAMLError as e:
		print(e)
		exit()

	# Open files to insert generated data
	param_gen_h = open("lib/param_generator/param_gen.h", "w")
	param_gen_c = open("lib/param_generator/param_gen.c", "w")
	param_gen_md = open("lib/param_generator/PARAMS.md", "w")

	# Prepare file headers

	str_md = "# Parameter File Reference\n\nName | Type | Description | Default | Unit | Options | Reboot\n--- | --- | --- | --- | --- | --- | ---\n"

	# Write file headers
	param_gen_md.write(str_md)

	try:
		# Iterate through every parameter
		for p in params.items():
			# Sanity Checks

			if p[1]["value"]["option"] == "scalar":
				param_type = p[1]["type"]
				param_val = p[1]["value"]["default"]
				param_val_type = type(p[1]["value"]["default"])
				raise_error = False

				if param_type == "float":
					if param_val_type != float:
						raise_error = True
				elif param_type == "int":
					if param_val_type != int:
						raise_error = True
				elif param_type == "uint":
					if (param_val_type != int) or (param_val < 0):
						raise_error = True

				if raise_error:
					raise ValueError("%s: Type (%s) and default value (%s) mismatch" % (p[0], param_type, str(param_val_type)))
			elif p[1]["value"]["option"] == "range":
				# TODO:
				# default is within range
				# default right type
				# range is in same units as default
				# max > min
				pass
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

			if p[1]["value"]["option"] == "scalar":
				str_md += str(p[1]["value"]["default"])
				str_md += " :| "
				# TODO:
				#str_md = p[1]["value"]["unit"]
				str_md += " | "
				str_md += "scalar"
				str_md += " | "
			elif p[1]["value"]["option"] == "range":
				str_md += str(p[1]["value"]["default"])
				str_md += " :| "
				#TODO:
				#str_md = p[1]["value"]["unit"]
				str_md += " | "
				str_md += "[min:%s, max:%s]" % (str(p[1]["value"]["min"]), str(p[1]["value"]["max"]))
				str_md += " | "
			elif p[1]["value"]["option"] == "list":
				str_md += str(p[1]["value"]["list"][p[1]["value"]["default"]])
				str_md += " :| "
				#TODO:
				#str_md = p[1]["value"]["unit"]
				str_md += " | "
				str_md += "[%s]" % str(p[1]["value"]["list"])
				str_md += " | "
			elif p[1]["value"]["option"] == "generated":
				# TODO: More detail from default and unit values?
				#str_md += str(p[1]["value"]["default"])
				str_md += " | "
				#str_md = p[1]["value"]["unit"]
				str_md += " | "
				str_md += "None"
				str_md += " | "
			else:
				raise ValueError("%s: Unsupported value option (%s)" % (p[0], p[1]["value"]["option"]))


			str_md += str(p[1]["reboot"])

			str_md += "\n"

			param_gen_md.write(str_md)

	except Exception as e:
		print("Error: %s" % e)
		exit()
	# Prepare file footers
	#str_md = ""

	# Write file footers
	#param_gen_md.write(str_md)

	# Close generated files
	param_gen_h.close()
	param_gen_c.close()
	param_gen_md.close()
