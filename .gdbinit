# Ensure correct cwd
cd /workspaces/HighStakes
python
# prevents vscode from being silly and running this multiple times
if 'rerunGuard' not in globals():
  rerunGuard = True

  print("Adding hot and cold symbol files")
  gdb.execute("add-symbol-file ./bin/sim/hot.package.elf")
  gdb.execute("add-symbol-file ./bin/sim/cold.package.elf")
  print("Added hot and cold symbol files")

  # add kernel symbol file
  import os
  print("Adding kernel symbol file")
  gdb.execute("add-symbol-file \"{}\"".format(os.environ["V5_SIM_KERNEL_PATH"]))
  print("Added kernel symbol file")
  
  print("Adding dependency source dirs")
  gdb.execute(
    "dir \"$cdir:$cwd:{v5_sim}:{templates}:{kernel}\"".format(
      v5_sim=os.environ["V5_SIM_SOURCE_PATH"],
      templates=os.environ["TEMPLATES_SOURCE_PATH"],
      kernel=os.environ["TEMPLATES_SOURCE_PATH"] + "/kernel"
  ))
  print("Added dependency source dirs")

  print("Adding pretty printers")
  import sys
  sys.path.insert(0, '.')
  from printers import register_eigen_printers
  register_eigen_printers(None)
  print("Added pretty printers")

  print("Adding pros kernel substitute path")
  gdb.execute(
      "set substitute-path C:/Users/noama/Documents/pros/pros {template_path}/kernel".format(
          template_path=os.environ["TEMPLATES_SOURCE_PATH"]))
  print("Added pros kernel substitute path")

  gdb.execute("alias connect = target remote localhost:1234")
  gdb.execute("alias b_pros_init = b /.templates/kernel/src/system/startup.c:36")
  
  gdb.execute("define continue_and_print_reg3\ncontinue\nx/i $r3\nend")

  # Add freertos helper command
  # install with `pip install freertos-gdb`
  import freertos_gdb

  print("Added helper commands")
  # attach to remote target
  # gdb.execute("target remote localhost:1234")
end