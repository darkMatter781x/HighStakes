# Ensure correct cwd
cd /workspaces/HighStakes
python
# prevents vscode from being silly and running this multiple times
if 'rerunGuard' not in globals():
  rerunGuard = True

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
    "dir \"$cdir:$cwd:{v5_sim}:{templates}\"".format(
      v5_sim=os.environ["V5_SIM_SOURCE_PATH"],
      templates=os.environ["TEMPLATES_SOURCE_PATH"]
  ))
  
  print("Added dependency source dirs")

  # attach to remote target
  # gdb.execute("target remote localhost:1234")
end