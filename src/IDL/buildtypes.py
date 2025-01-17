#!/usr/bin/python3

import os
import subprocess
import shutil

script_base_dir = os.getcwd()
print("Recreating directories \"AUTOGENERATED\" and \"M12Typelib\"")
subprocess.call(['rm', '-rf', 'AUTOGENERATED'])
subprocess.call(['rm', '-rf', 'M12Typelib'])


cmd = 'rtiddsgen'
platform = 'x64Linux4gcc7.3.0'
platform_arg = " -platform " + platform
language = "C++11"
language_arg = " -language " + language
update_arg = " -autoGenFiles " + platform
#constructor_arg = " -constructor"
constructor_arg = ""
enumarg = " -qualifiedEnumerator "

final_command = cmd + platform_arg + language_arg + update_arg + constructor_arg + enumarg
print(final_command)

subprocess.call(['mkdir', 'AUTOGENERATED'])
autogen_dir = script_base_dir + os.path.sep + 'AUTOGENERATED'
print(autogen_dir)

generated_types = []
print("")
print("Generating and compiling the DDS types")
print("following outputs are from commands \"rtiddsgen\" and \"make\"")
print("--------------------------------------------------------------------------------")
print("")
for root,dirs,files in os.walk(script_base_dir):
    for file in files:
        if file.endswith('.idl'):
            #print(os.path.join(root,file))
            generated_type_dirname = '' + os.path.splitext(file)[0]
            #print(generated_type_dirname)
            os.chdir(autogen_dir)
            subprocess.call(['mkdir', generated_type_dirname])
            generated_type_dir = os.getcwd() + os.path.sep + generated_type_dirname
            os.chdir(generated_type_dir)


            #print(os.path.join(root,file))
            cmd = final_command + ' -inputIDL ' + os.path.join(root,file) + ' -d ' + os.getcwd()
            os.system(cmd)
            make_cmd = 'make -f ' + 'makefile_' + generated_type_dirname + '_' + platform
            os.system(make_cmd)

            generated_types.append(generated_type_dirname)

# Report to a file which types were generated
print("")
print("--------------------------------------------------------------------------------")
print("Finished generating and compiling the DDS types")
print("")
#print("Creating a file named \" generated_types.txt \"  describing which .idl files were generated and compiled...")
os.chdir(script_base_dir)
#with open('generated_types.txt', 'w') as f:
#    for item in generated_types:
#        f.write("%s\n" % item)

subprocess.call(['mkdir', 'M12Typelib'])
os.chdir(script_base_dir + os.path.sep + 'M12Typelib')
subprocess.call(['mkdir', 'include'])
subprocess.call(['mkdir', 'objs'])

print("")
print("Starting copying the files to the M12Typelib directory...")
print("")
os.chdir(script_base_dir)
for dirpath, dirnames, filenames in os.walk('.'):
    for filename in [f for f in filenames if f.endswith(".o")]:
        print("Copying following object file: ")
        print(os.path.join(dirpath,filename))
        print("")
        shutil.copyfile(os.path.join(dirpath,filename), os.getcwd() + os.path.sep + 'M12Typelib' + os.path.sep + 'objs' + os.path.sep + filename )

os.chdir(script_base_dir)
for dirpath, dirnames, filenames in os.walk('.'):
    for filename in [f for f in filenames if f.endswith((".hpp",".h"))]:
        print("Copying following header file: ")
        print(os.path.join(dirpath,filename))
        print("")
        shutil.copyfile(os.path.join(dirpath,filename), os.getcwd() + os.path.sep + 'M12Typelib' + os.path.sep + 'include' + os.path.sep + filename )