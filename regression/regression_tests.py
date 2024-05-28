import os
import sys

#arguments:
# 0 - script name
# 1 - sofa build dir
#use: 
# python regression_tests.py /c/projects/sofa-build/

sofa_build_dir = sys.argv[1]
pwd = os.getcwd()
reg_binary = sofa_build_dir + '/bin/Release/Regression_test.exe'
ref_dir = pwd + "/references/"
scenes_dir = pwd + "/../scenes/"

# logs
print("--- Regression config ---")
print("sofa_build_dir: " + sofa_build_dir)
print("ref_dir: " + ref_dir)
print("scenes_dir: " + scenes_dir)
print("reg_binary: " + reg_binary)
print("-------------------------")

os.environ["REGRESSION_SCENES_DIR"] = scenes_dir
os.environ["REGRESSION_REFERENCES_DIR"] = ref_dir

os.environ["SOFA_ROOT"] = sofa_build_dir
os.environ["SOFA_PLUGIN_PATH"] = sofa_build_dir + '/lib'

os.system(reg_binary)
