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

	# Iterate through every parameter
	for p in params.items():
		str_md = p[1]["name"]
		str_md += " | "
		str_md += p[1]["type"]
		str_md += " | "
		str_md += p[1]["description"]
		str_md += " | "

		if p[1]["value"]["option"] == "scalar":
			str_md += str(p[1]["value"]["default"])
			str_md += " | "
			#str_md = p[1]["value"]["unit"]
			str_md += " | "
			str_md += p[1]["value"]["option"]
			str_md += " | "
		else:
			str_md += "| | | "


		str_md += str(p[1]["reboot"])

		str_md += "\n"

		param_gen_md.write(str_md)

	# Prepare file footers
	#str_md = ""

	# Write file footers
	#param_gen_md.write(str_md)

	# Close generated files
	param_gen_h.close()
	param_gen_c.close()
	param_gen_md.close()
