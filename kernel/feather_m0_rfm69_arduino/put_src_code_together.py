import os
from glob import glob

src_dir = "src"
target_c_file = "sketchbook/main/main.ino"
target_h_file = "sketchbook/main/main.h"

def fix_c_code(code):
    fixed_code = ""
    for line in code.splitlines():
        if "#include" in line:
            #ignore includes, since we're moving everything to a
            #single file
            print("  -removed {}".format(line))
        else:
            fixed_code += line
            fixed_code += "\n"


    return fixed_code

def fix_c_code_main(code):
    fixed_code = ""
    for line in code.splitlines():
        if "#include" in line:
            #Replace include for main header file
            header_name = target_h_file.split("/")[1]
            fixed_line = '#include "{}.h"'.format(header_name)
            fixed_code += fixed_line
            print("  -replaced {} for {}".format(line, fixed_line))
        else:
            fixed_code += line

        fixed_code += "\n"


    return fixed_code



#Collect filenames
#
# https://stackoverflow.com/questions/18394147/recursive-sub-folder-search-and-return-files-in-a-list-python
#
print("Recursively Collecting paths in {}".format(src_dir))
paths = [y for x in os.walk("src") for y in glob(os.path.join(x[0], '*'))]

#Split and Clean
#(remove directories + main.c)
clean_c_paths = []
clean_h_paths = []
for path in paths:
    if os.path.isfile(path) and "main.c" not in path:
        if ".c" in path:
            clean_c_paths.append(path)
        elif ".h" in path:
            clean_h_paths.append(path)

#Read contents of source code
c_contents = ""

#Main first
print("")
print("Reading {}/main.c".format(src_dir))
with open("{}/main.c".format(src_dir), "r") as reader:
    c_contents += fix_c_code_main(reader.read())
    c_contents += "\n\n\n"

#Now the rest
for path in clean_c_paths:
    print("Reading {}".format(path))
    with open(path, "r") as reader:
        c_contents += fix_c_code(reader.read())
        c_contents += "\n\n\n"

#Write them as a single file
print("Writing everything to {}".format(target_c_file))
with open("{}".format(target_c_file), "w") as writer:
    writer.write(c_contents)

#Read contents of header files
h_contents = ""

print("")
for path in clean_h_paths:
    print("Reading {}".format(path))
    with open(path, "r") as reader:
        h_contents += reader.read()
        h_contents += "\n\n\n"

#Write them as a single file
print("Writing everything to {}".format(target_h_file))
with open("{}".format(target_h_file), "w") as writer:
    writer.write(h_contents)
