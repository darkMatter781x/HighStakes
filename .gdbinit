# Ensure correct cwd
cd /workspaces/HighStakes
python
# prevents vscode from being silly and running this multiple times
if 'rerunGuard' not in globals():
  rerunGuard = True

  # add matrix pretty printer
  import sys
  sys.path.insert(0, './eigen-printers')

  from printers import register_eigen_printers, register_commands
  register_eigen_printers(None)
  register_commands()
  print("Eigen pretty printers registered")

  print("Adding monolith symbol file")
  gdb.execute("add-symbol-file ./bin/monolith.elf")
  print("Added monolith symbol file")

  # add kernel symbol file
  import os
  print("Adding kernel symbol file")
  gdb.execute("add-symbol-file \"{}\"".format(os.environ["V5_SIM_KERNEL_PATH"]))
  print("Added kernel symbol file")
  
  print("Adding dependency source dirs")
  gdb.execute(
    "dir \"$cdir:$cwd:{v5_sim}:{pros}\"".format(
      v5_sim=os.environ["V5_SIM_SOURCE_PATH"],
      pros=os.environ["PROS_SOURCE_PATH"]
  ))
  
  print("Added dependency source dirs")

  # attach to remote target
  # gdb.execute("target remote localhost:1234")
end