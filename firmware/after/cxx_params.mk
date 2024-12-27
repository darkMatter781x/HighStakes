# Override the cxx rule and add the ability to specify flags per file
# TODO: Extend this to other rules


$(info "overriding cxx rule")

# At the top of each file, you can specify params to be used in the compilation of that file
# These are specified in the form of /// PARAM_NAME=VALUE
# Any line that does not start with a /// terminates the param list
# This function gets the value of a param (named $2) from the top of a file ($1)
GET_FILE_PARAM=$(shell sed -nE -e '1,/^([^/\n].?.?|.[^/\n].?|..[^/\n])/ { \%^///%p }' "$1" | sed -E 's%^///\s*%%' | sed -n '/^$2=/s/$2=//p')

# This function gets the extra flags for a specific file from its params
GET_FILE_FLAGS=$(call GET_FILE_PARAM,$1,EXTRA_FLAGS)
define cxx_rule
$(BINDIR)/%.$1.o: $(SRCDIR)/%.$1
$(BINDIR)/%.$1.o: $(SRCDIR)/%.$1 $(DEPDIR)/$(basename %).d
	$$(eval $$@_EXTRA_FLAGS=$$$$(call GET_FILE_FLAGS,$$<))
	$(VV)mkdir -p $$(dir $$@)
	$(MAKEDEPFOLDER)
	$$(info Compiling $$< with: $(CXX) -c $(INCLUDE) -iquote"$(INCDIR)/$$(dir $$*)" $(CXXFLAGS) $(EXTRA_CXXFLAGS) $$($$@_EXTRA_FLAGS) $(DEPFLAGS) -o $$@ $$<)
	$$(call test_output_2,Compiled $$< ,$(CXX) -c $(INCLUDE) -iquote"$(INCDIR)/$$(dir $$*)" $(CXXFLAGS) $(EXTRA_CXXFLAGS) $$($$@_EXTRA_FLAGS) $(DEPFLAGS) -o $$@ $$<,$(OK_STRING))
	$(RENAMEDEPENDENCYFILE)
endef
$(foreach cxxext,$(CXXEXTS),$(eval $(call cxx_rule,$(cxxext))))