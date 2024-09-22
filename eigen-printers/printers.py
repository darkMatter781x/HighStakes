# -*- coding: utf-8 -*-
# This file is part of Eigen, a lightweight C++ template library
# for linear algebra.
#
# Copyright (C) 2009 Benjamin Schindler <bschindler@inf.ethz.ch>
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

# Pretty printers for Eigen::Matrix
# This is still pretty basic as the python extension to gdb is still pretty basic.
# It cannot handle complex eigen types and it doesn't support many of the other eigen types
# This code supports fixed size as well as dynamic size matrices

# To use it:
#
# * Create a directory and put the file as well as an empty __init__.py in
#   that directory.
# * Create a ~/.gdbinit file, that contains the following:
#      python
#      import sys
#      sys.path.insert(0, '/path/to/eigen/printer/directory')
#      from printers import register_eigen_printers
#      register_eigen_printers(None)
#      end

log_file = None
def merge_dicts(*dict_args):
    """
    Given any number of dictionaries, shallow copy and merge into a new dict,
    precedence goes to key-value pairs in latter dictionaries.
    """
    result = {}
    for dictionary in dict_args:
        result.update(dictionary)
    return result

def register_commands():
    class WriteConfig (gdb.Command):
        """Greet the whole world."""

        def __init__ (self):
            super (WriteConfig, self).__init__ ("conf", gdb.COMMAND_USER, prefix=True)

        def invoke (self, arg, from_tty):
            try:
                args = gdb.string_to_argv(arg)
                file_name = args[0]
                gdb_command = args[1:]
                out_file = None
                try:
                    out_file = open(file_name, "w")
                except Exception as err:
                    print("failed to open config dump file: ", err)
                out_file.write(gdb.execute(" ".join(gdb_command), False, True))
                out_file.close()
                print("wrote to file: {file_name}".format(file_name=file_name))
            except Exception as err:
                print("failed to write config: ", err)
    WriteConfig()

    class WriteAllConfig (gdb.Command):
        """Greet the whole world."""

        def __init__ (self):
            super (WriteAllConfig, self).__init__ ("conf all", gdb.COMMAND_USER)

        def invoke (self, arg, from_tty):
            import re
            import json

            pattern = re.compile('^show .+ --')
            def find_all_show_subcommands(cmd):
                if cmd == None:
                    cmd = "show"
                sub_most_cmd = cmd.split(" ")[-1]
                arr = [
                    find_all_show_subcommands(line.split(" --")[0].split(",")[0])
                    for line
                    in gdb.execute("help " + cmd, False, True).split("\n")
                    if pattern.match(line)
                ]
                if len(arr) == 0:
                    return sub_most_cmd
                return (sub_most_cmd, arr)
            
            def get_show_config(cmd, parents=[]):
                if type(cmd) == str:
                    return {cmd: gdb.execute(" ".join(parents) + " " + cmd , False, True)}
                if type(cmd) == tuple:
                    return {cmd[0]: merge_dicts(*(get_show_config(subcmd, parents=parents + [cmd[0]]) for subcmd in cmd[1])) }

            try:
                file_name = arg
                out_file = None
                
                shows = find_all_show_subcommands(None)
                result_dict = get_show_config(shows)["show"]
                result_json = json.dumps(result_dict, indent=2)
                try:
                    out_file = open(file_name, "w")
                except Exception as err:
                    print("failed to open config dump file: ", err)
                
                out_file.write(str(result_json))
                out_file.close()

                print("wrote to file: {file_name}".format(file_name=file_name))
            except Exception as err:
                print("failed to create config file: ", err)
    WriteAllConfig()

def log(*a):
    pass
    # global log_file
    # if log_file == None:
    #     try:
    #         log_file = open("eigen.gdb.log", "w")
    #     except Exception as err:
    #         print("failed to open log file: ", err)
    # print(a)
    # log_file.write(str(a))
    # log_file.write("\n")
    # log_file.flush()


try:
    import gdb
    import re
    from bisect import bisect_left

    # Basic row/column iteration code for use with Sparse and Dense matrices
    class _MatrixEntryIterator(object):

        def __init__(self, rows, cols, row_major):
            self.rows = rows
            self.cols = cols
            self.currentRow = 0
            self.currentCol = 0
            self.rowMajor = row_major

        def __iter__(self):
            return self

        def next(self):
            return self.__next__()  # Python 2.x compatibility

        def __next__(self):
            row = self.currentRow
            col = self.currentCol
            if self.rowMajor == 0:
                if self.currentCol >= self.cols:
                    raise StopIteration

                self.currentRow += 1
                if self.currentRow >= self.rows:
                    self.currentRow = 0
                    self.currentCol += 1
            else:
                if self.currentRow >= self.rows:
                    raise StopIteration

                self.currentCol += 1
                if self.currentCol >= self.cols:
                    self.currentCol = 0
                    self.currentRow += 1

            return row, col

    class EigenMatrixPrinter:
        """Print Eigen Matrix or Array of some kind"""

        def __init__(self, variety, val):
            """Extract all the necessary information"""

            # Save the variety (presumably "Matrix" or "Array") for later usage
            self.variety = variety

            # The gdb extension does not support value template arguments - need to extract them by hand
            typeinfo = val.type
            if typeinfo.code == gdb.TYPE_CODE_REF:
                typeinfo = typeinfo.target()
            self.type = typeinfo.unqualified().strip_typedefs()
            tag = self.type.tag
            regex = re.compile("<.*>")
            m = regex.findall(tag)[0][1:-1]
            template_params = m.split(",")
            template_params = [x.replace(" ", "") for x in template_params]

            if template_params[1] in ["-0x00000000000000001", "-0x000000001", "-1"]:
                self.rows = val["m_storage"]["m_rows"]
            else:
                self.rows = int(template_params[1])

            if template_params[2] in ["-0x00000000000000001", "-0x000000001", "-1"]:
                self.cols = val["m_storage"]["m_cols"]
            else:
                self.cols = int(template_params[2])

            self.options = 0  # default value
            if len(template_params) > 3:
                self.options = template_params[3]

            self.rowMajor = int(self.options) & 0x1

            self.innerType = self.type.template_argument(0)

            self.val = val

            # Fixed size matrices have a struct as their storage, so we need to walk through this
            self.data = self.val["m_storage"]["m_data"]
            if self.data.type.code == gdb.TYPE_CODE_STRUCT:
                self.data = self.data["array"]
                self.data = self.data.cast(self.innerType.pointer())

        class _Iterator(_MatrixEntryIterator):
            def __init__(self, rows, cols, data_ptr, row_major):
                super(EigenMatrixPrinter._Iterator, self).__init__(
                    rows, cols, row_major
                )

                self.dataPtr = data_ptr

            def __next__(self):
                row, col = super(EigenMatrixPrinter._Iterator, self).__next__()

                item = self.dataPtr.dereference()
                self.dataPtr += 1
                if self.cols == 1:  # if it's a column vector
                    return "[%d]" % (row,), item
                elif self.rows == 1:  # if it's a row vector
                    return "[%d]" % (col,), item
                return "[%d,%d]" % (row, col), item

        def to_string(self):
            log("(%d, %d): casting to string" % (self.rows, self.cols))
            import json
            data_list = list(self._Iterator(self.rows, self.cols, self.data, self.rowMajor))
            data = dict()
            for entry in data_list:
                key_str = entry[:1][0]
                key = tuple(int(s) for s in key_str[1:-1].split(","))
                if len(key) == 1:
                    key += (0,)
                data[key] = float(entry[1:][0])
            epsilon = pow(2,-23)
            def float_to_str(flt: float):
                if flt != 0 and abs(flt) < epsilon:
                    return "~" + str(0.0)
                return str(flt)
            log("Constructing out dict from data: ", data)

            out = {
                "kind": {"grid": True},
                "rows": [{
                    "columns": [
                        {
                            "content": float_to_str(float(data[(y,x)]))
                        }
                        for x in range(self.cols) 
                    ]
                } for y in range(self.rows)],
            }
            log("Constructing out str from out: ")
            log(out)
            out_str = json.dumps(out)
            log(out_str)
            return out_str

    class EigenSparseMatrixPrinter:
        """Print an Eigen SparseMatrix"""

        def __init__(self, val):
            """Extract all the necessary information"""

            typeinfo = val.type
            if typeinfo.code == gdb.TYPE_CODE_REF:
                typeinfo = typeinfo.target()
            self.type = typeinfo.unqualified().strip_typedefs()
            tag = self.type.tag
            regex = re.compile("<.*>")
            m = regex.findall(tag)[0][1:-1]
            template_params = m.split(",")
            template_params = [x.replace(" ", "") for x in template_params]

            self.options = 0
            if len(template_params) > 1:
                self.options = template_params[1]

            self.rowMajor = int(self.options) & 0x1

            self.innerType = self.type.template_argument(0)

            self.val = val

            self.data = self.val["m_data"]
            self.data = self.data.cast(self.innerType.pointer())

        class _Iterator(_MatrixEntryIterator):
            def __init__(self, rows, cols, val, row_major):
                super(EigenSparseMatrixPrinter._Iterator, self).__init__(
                    rows, cols, row_major
                )

                self.val = val

            def __next__(self):
                row, col = super(EigenSparseMatrixPrinter._Iterator, self).__next__()

                # repeat calculations from SparseMatrix.h:
                outer = row if self.rowMajor else col
                inner = col if self.rowMajor else row
                start = self.val["m_outerIndex"][outer]
                end = (
                    (start + self.val["m_innerNonZeros"][outer])
                    if self.val["m_innerNonZeros"]
                    else self.val["m_outerIndex"][outer + 1]
                )

                # and from CompressedStorage.h:
                data = self.val["m_data"]
                if start >= end:
                    item = 0
                elif (end > start) and (inner == data["m_indices"][end - 1]):
                    item = data["m_values"][end - 1]
                else:
                    # create Python index list from the target range within m_indices
                    indices = [
                        data["m_indices"][x] for x in range(int(start), int(end) - 1)
                    ]
                    # find the index with binary search
                    idx = int(start) + bisect_left(indices, inner)
                    if idx < end and data["m_indices"][idx] == inner:
                        item = data["m_values"][idx]
                    else:
                        item = 0

                return "[%d,%d]" % (row, col), item

        def children(self):
            if self.data:
                return self._Iterator(self.rows(), self.cols(), self.val, self.rowMajor)

            return iter([])  # empty matrix, for now

        def rows(self):
            return self.val["m_outerSize"] if self.rowMajor else self.val["m_innerSize"]

        def cols(self):
            return self.val["m_innerSize"] if self.rowMajor else self.val["m_outerSize"]

        def to_string(self):

            if self.data:
                status = (
                    "not compressed" if self.val["m_innerNonZeros"] else "compressed"
                )
            else:
                status = "empty"
            dimensions = "%d x %d" % (self.rows(), self.cols())
            layout = "row" if self.rowMajor else "column"

            return "Eigen::SparseMatrix<%s>, %s, %s major, %s" % (
                self.innerType,
                dimensions,
                layout,
                status,
            )

    class EigenQuaternionPrinter:
        """Print an Eigen Quaternion"""

        def __init__(self, val):
            """Extract all the necessary information"""
            # The gdb extension does not support value template arguments - need to extract them by hand
            typeinfo = val.type
            if typeinfo.code == gdb.TYPE_CODE_REF:
                typeinfo = typeinfo.target()
            self.type = typeinfo.unqualified().strip_typedefs()
            self.innerType = self.type.template_argument(0)
            self.val = val

            # Quaternions have a struct as their storage, so we need to walk through this
            self.data = self.val["m_coeffs"]["m_storage"]["m_data"]["array"]
            self.data = self.data.cast(self.innerType.pointer())

        class _Iterator:
            def __init__(self, data_ptr):
                self.dataPtr = data_ptr
                self.currentElement = 0
                self.elementNames = ["x", "y", "z", "w"]

            def __iter__(self):
                return self

            def next(self):
                return self.__next__()  # Python 2.x compatibility

            def __next__(self):
                element = self.currentElement

                if self.currentElement >= 4:  # there are 4 elements in a quaternion
                    raise StopIteration

                self.currentElement += 1

                item = self.dataPtr.dereference()
                self.dataPtr += 1
                return "[%s]" % (self.elementNames[element],), item

        def children(self):
            return self._Iterator(self.data)

        def to_string(self):
            return "Eigen::Quaternion<%s> (data ptr: %s)" % (self.innerType, self.data)

    def cast_eigen_block_to_matrix(val):
        # Get the type of the variable (and convert to a string)
        # Example: 'const Eigen::Block<Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, -1, -1, false> const, -1, -1, false>'
        val_type = str(val.type)

        # Extract the Eigen::Matrix type from the Block:
        # From the previous example: Eigen::Matrix<double, -1, -1, 0, -1, -1>
        begin = val_type.find("Eigen::Matrix<")
        end = val_type.find(">", begin) + 1

        # Convert the Eigen::Block to an Eigen::Matrix
        return val.cast(gdb.lookup_type(val_type[begin:end]))

    def build_eigen_dictionary():
        pretty_printers_dict[re.compile("^Eigen::Quaternion<.*>$")] = (
            lambda val: EigenQuaternionPrinter(val)
        )
        pretty_printers_dict[re.compile("^Eigen::Matrix<.*>$")] = (
            lambda val: EigenMatrixPrinter("Matrix", val)
        )
        pretty_printers_dict[re.compile("^Eigen::Block<.*>$")] = (
            lambda val: EigenMatrixPrinter("Matrix", cast_eigen_block_to_matrix(val))
        )
        pretty_printers_dict[re.compile("^Eigen::VectorBlock<.*>$")] = (
            lambda val: EigenMatrixPrinter("Matrix", cast_eigen_block_to_matrix(val))
        )
        pretty_printers_dict[re.compile("^Eigen::SparseMatrix<.*>$")] = (
            lambda val: EigenSparseMatrixPrinter(val)
        )
        pretty_printers_dict[re.compile("^Eigen::Array<.*>$")] = (
            lambda val: EigenMatrixPrinter("Array", val)
        )

    def register_eigen_printers(obj):
        """Register eigen pretty-printers with objfile Obj"""
        if obj is None:
            obj = gdb
        obj.pretty_printers.append(lookup_function)

    def lookup_function(val):
        """Look-up and return a pretty-printer that can print val."""
        # log("lookup_function: ", val)

        typeinfo = val.type

        if typeinfo.code == gdb.TYPE_CODE_REF:
            typeinfo = typeinfo.target()

        typeinfo = typeinfo.unqualified().strip_typedefs()

        typename = typeinfo.tag
        if typename is None:
            return None

        for function in pretty_printers_dict:
            if function.search(typename):
                return pretty_printers_dict[function](val)

        return None

    pretty_printers_dict = {}

    build_eigen_dictionary()
except Exception as err:
    log(err)
