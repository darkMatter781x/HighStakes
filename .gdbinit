python
import sys
sys.path.insert(0, '/workspaces/HighStakes/eigen-printers')
from printers import register_eigen_printers
register_eigen_printers(None)
end