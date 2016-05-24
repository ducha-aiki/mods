#automatically generated makefile
packagename := daisy
major_version := 1
minor_version := 8
tiny_version := 0
author := Engin Tola
description := daisy descriptor
licence := see license.txt
#........................................
installdir := /home/tola/usr
external_sources :=
external_libraries :=
libdir := lib
srcdir := .
includedir:= include
define_flags :=
#........................................
optimize := true
parallelize := false
f77 := false
sse := false
multi-threading := false
profile := false
#........................................
specialize := false
platform := native
#........................................
sources := src/daisy.cpp src/main.cpp src/image_io_bmp.cpp src/image_io_png.cpp src/image_io_pnm.cpp src/image_io_jpeg.cpp src/image_manipulation.cpp src/progress_bar.cpp src/interaction.cpp src/general.cpp src/corecv.cpp

################################################################################
################# - MAKEFILE STATIC VARIABLES - ################################
################################################################################

ifeq ($(major_version),)
 major_version := 0
endif
ifeq ($(minor_version),)
 minor_version := 1
endif
ifeq ($(tiny_version),)
 tiny_version := 0
endif

ifeq ($(version),)
 version := $(major_version)"."$(minor_version)"."$(tiny_version)
endif

ifeq ($(optimize),false)
  packagename := $(packagename)d
  outdir := debug/
else
  outdir := release/
endif

srcdir := ${srcdir}/

sources_list = $(addprefix $(srcdir), $(sources))

objectfiles       := $(filter %.o,$(subst   .c,.o,$(sources)))
objectfiles       += $(filter %.o,$(subst  .cc,.o,$(sources)))
objectfiles       += $(filter %.o,$(subst .cpp,.o,$(sources)))

objects = $(addprefix $(outdir), $(objectfiles))

dependencies  := $(subst .o,.d,$(objects))

depdir := $(dir ${dependencies})

bindir := ${outdir}'/bin/'
libdir := ${outdir}${libdir}
libname       := lib$(packagename)
libtarget     := $(libdir)/$(libname).a
libsoname     := $(libname).so.$(major_version)
librealname   := $(libdir)/$(libname).so.$(version)
exetarget     := ${bindir}$(packagename)
pkgconfigfile := $(packagename).pc

automakefile := make.auto
commentfile  := makefile.comment

tag_file:=TAGS
tag_generator:='$(MAKEFILE_HEAVEN)/tags.sh'
tag_depends:=${external_libraries}
tag_src := ${includedir}/*.h ${includedir}/${packagename}/*.h		\
${includedir}/*.tcc ${includedir}/${packagename}/*.tcc ${srcdir}*.cpp	\
${srcdir}*.cc $(srcdir)*.c

compiler := g++
# compiler := colorgcc
CXX := ${compiler}

curpath=`pwd -P`

REQUIRED_DIRS = ${outdir} ${libdir} ${depdir} ${bindir}

################################################################################
########################### - MAKEFILE FLAGS - #################################
################################################################################

ARFLAGS = ruv
CTAGFLAGS := -e -R --languages=c++,c

CXXFLAGS += -ffast-math  ${define_flags} -I$(includedir) ${custom_cflags}
LDFLAGS += ${custom_ld_flags}

ifeq ($(optimize),false)
  external_libraries := $(subst argus,argusd,$(external_libraries))
  external_libraries := $(subst kutility,kutilityd,$(external_libraries))
  external_libraries := $(subst kortex,kortexd,$(external_libraries))
  external_libraries := $(subst karpet,karpetd,$(external_libraries))
  external_libraries := $(subst daisy,daisyd,$(external_libraries))
  external_libraries := $(subst evidence,evidenced,$(external_libraries))
  external_libraries := $(subst kost,kostd,$(external_libraries))
endif

ifneq ($(external_sources),)
 CXXFLAGS += `pkg-config --cflags ${external_sources}`
endif

ifneq ($(external_libraries),)
 CXXFLAGS += `pkg-config --cflags ${external_libraries}`
 LDFLAGS  += `pkg-config --cflags-only-other --libs ${external_libraries}`
endif

ifeq ($(f77),true)
 LDFLAGS += -lg2c
endif

ifeq ($(boost-thread),true)
 LDFLAGS += -lboost_thread-mt
endif

ifeq ($(sse),true)
    CXXFLAGS += -msse -msse4.2 -DWITH_SSE
    CPPFLAGS += -msse -msse4.2 -DWITH_SSE
endif

CXXFLAGS += -fno-strict-aliasing -Wall -fPIC

ifeq ($(multi-threading),true)
    CXXFLAGS += -lpthread
endif

ifeq ($(profile),true)
  CXXFLAGS+= -pg
endif

dbg_flags = -g -DDEBUG
opt_flags = -O3 -DHAVE_INLINE -DNDEBUG
spc_flags = -march=$(platform) -mfpmath=sse

ifeq ($(optimize),true)
  CXXFLAGS += $(opt_flags)
  ifeq ($(specialize),true)
     CXXFLAGS += $(spc_flags)
  endif
else
  CXXFLAGS += ${dbg_flags}
  parallelize = false
  profile = true
endif

ifeq ($(parallelize),true)
   CXXFLAGS += -fopenmp
endif

is_debug := $(wildcard .debug)

ifeq ($(strip $(is_debug)),)
  is_debug = true
else
  is_debug = false
endif

ifeq ($(optimize),true)
  state_file = .release
else
  state_file = .debug
endif

################################################################################
################################ - MAKEFILE RULES - ############################
################################################################################

_MKDIRS := $(shell mkdir -p ${REQUIRED_DIRS})

.PHONY       : $(exetarget)
$(exetarget) : ${objects}
	@echo compiler path = ${compiler}
	@echo
	@echo ------------------ making executable
	@echo
	$(compiler) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

.PHONY : flags
flags :
	@echo
	@echo ------------------ build flags
	@echo
	@echo ldflags  = $(LDFLAGS)
	@echo
	@echo cxxflags = $(CXXFLAGS)
	@echo
	@echo source_list = ${sources_list}
	@echo
	@echo objects = ${objects}
	@echo
	@echo dependencies = ${dependencies}
	@echo depdir = ${depdir}

.PHONY : internal_var
internal_var :
	@echo {curpath}       ${curpath}
	@echo {compiler}      ${compiler}
	@echo {CXX}           ${CXX}
	@echo {libname}       ${libname}
	@echo {libtarget}     ${libtarget}
	@echo {libsoname}     ${libsoname}
	@echo {librealname}   ${librealname}
	@echo {exetarget}     ${exetarget}
	@echo {pkgconfigfile} ${pkgconfigfile}
	@echo {automakefile}  ${automakefile}
	@echo {commentfile}   ${commentfile}
	@echo {tag_file}      ${tag_file}
	@echo {tag_generator} ${tag_generator}
	@echo {tag_depends}   ${tag_depends}
	@echo {tag_src}       ${tag_src}
	@echo
	@echo ldflags  = $(LDFLAGS)
	@echo cxxflags = $(CXXFLAGS)
	@echo source_list = ${sources_list}
	@echo objects = ${objects}
	@echo dependencies = ${dependencies}
	@echo

.PHONY : compilation
compilation:
ifeq ($(is_debug),true)
	@echo debug
else
	@echo release
endif

.PHONY  : slib
slib   : $(objects)
	g++ -shared -Wl,-soname,$(libsoname) -o $(librealname)  $^
	ldconfig -v -n $(libdir)
	ln -sf $(libsoname) $(libdir)/$(libname).so

.PHONY  : library
library : $(libtarget) tags
	@echo
	@echo ------------------ library $(libtarget) is built.
	@echo

$(libtarget): $(objects)
	@echo
	@echo ------------------ creating library
	@echo
	$(AR) $(ARFLAGS) $@ $^

.PHONY : tags
tags   :
	@echo
	@echo ------------------ creating tag entries
	@echo
# 	@echo tin=$(tag_incl)
# 	@echo tag_depdends:${tag_depends}
# 	@echo tag_src:${tag_src}
	@${tag_generator} -l ${tag_depends} -f ${tag_src} -o ${tag_file}

.PHONY : dox
dox    : Doxyfile
	@echo
	@echo ------------------ creating documentation
	@echo
	@doxygen Doxyfile

.PHONY : clean
clean  :
	@echo
	@echo ------------------ cleaning *.o exe lib
	@echo
	@echo rm -f $(objects) $(libtarget) ${exetarget} $(tag_file) gmon.out $(librealname) $(libdir)/$(libname).so $(libdir)/$(libsoname)
	@rm -f $(objects) $(libtarget) ${libtarget} ${exetarget} $(tag_file) gmon.out $(librealname) $(libdir)/$(libname).so $(libdir)/$(libsoname)


.PHONY : cleanaux
cleanaux  :
	@echo
	@echo ------------------ cleaning *.o *.d
	@echo
	@echo rm -f $(objects) ${dependencies} $(tag_file) gmon.out
	@rm -f $(objects) ${dependencies} $(tag_file) gmon.out

.PHONY   : cleandox
cleandox :
	@echo
	@echo ------------------ removing documentation
	@echo
	@rm -rf doc

.PHONY : cleandist
cleandist  :
	@echo
	@echo ------------------ cleaning everything
	@echo
	@rm -f $(pkgconfigfile) $(libtarget) $(objects) ${exetarget} $(dependencies) $(tag_file) gmon.out  $(librealname) $(libdir)/$(libname).so $(libdir)/$(libsoname)

.PHONY   : cleandep
cleandep :
	@echo ------------------ cleaning dependencies
	@rm -rf ${dependencies}

.PHONY : clear
clear :
	@rm -rf \#* ${dependencies} *~

.PHONY: install-lib
install-lib: $(libtarget) tags pkgfile uninstall
	@echo
	@echo ------------------ installing library and header files
	@echo
	@echo ------------------ installing at $(installdir)
	@echo
	@mkdir -p $(installdir)/include
	@rsync -rv --exclude=.svn $(includedir)/* $(installdir)/include/
	@mkdir -p $(installdir)/lib/pkgconfig
	@cp -vfr $(libtarget)  $(installdir)/lib
	@echo
	@echo ------------------ installing the pkg-config file to $(installdir)/lib/pkgconfig. \
		Remember to add this path to your PKG_CONFIG_PATH variable
	@echo
	@cp $(pkgconfigfile) $(installdir)/lib/pkgconfig/

.PHONY: install-slib
install-slib: $(slib) tags pkgfile uninstall
	@echo
	@echo ------------------ installing library and header files
	@echo
	@echo ------------------ installing at $(installdir)
	@echo
	@mkdir -p $(installdir)/include
	@rsync -rv --exclude=.svn $(includedir)/* $(installdir)/include/
	@mkdir -p $(installdir)/lib/pkgconfig
	@cp -vfr $(libdir)/*  $(installdir)/lib
	@echo
	@echo ------------------ installing the pkg-config file to $(installdir)/lib/pkgconfig. \
		Remember to add this path to your PKG_CONFIG_PATH variable
	@echo
	@cp $(pkgconfigfile) $(installdir)/lib/pkgconfig/


.PHONY: install
install: $(exetarget) tags
	@cp -f $(exetarget) $(installdir)/bin

.PHONY: install-dev
install-dev : $(libtarget) pkgfile uninstall
	@echo
	@echo ------------------ installing library and development files
	@echo
	@echo ------------------ installing at $(installdir)
	@echo
	@mkdir -p $(installdir)/include/$(packagename)
	@echo ------------------ copying .h $(installdir)/include/
	@rsync -rv --exclude=.svn $(includedir)/* $(installdir)/include/
	@mkdir -p $(installdir)/lib/pkgconfig
	@cp -vfR $(libtarget)  $(installdir)/lib                 # copy the static library
	@mkdir -p $(installdir)/src/$(packagename)                 # create the source directory
	@rsync -rv --exclude=.svn $(srcdir)/* $(installdir)/src/$(packagename)
	@cp -vf makefile $(installdir)/src/$(packagename)
	@cp $(pkgconfigfile) $(installdir)/lib/pkgconfig/

.PHONY: uninstall
uninstall:
	@echo
	@echo ------------------ uninstalling if-installed
	@echo
	@rm -rf $(installdir)/include/$(packagename)
	@rm -f   $(installdir)/$(libtarget)
	@rm -rf $(installdir)/src/$(packagename)
	@rm -f   $(installdir)/lib/pkgconfig/$(pkgconfigfile)
	@rm -f   $(installdir)/bin/$(exetarget)
	@rm -f $(installdir)/lib/$(libsoname)
	@rm -f $(installdir)/$(librealname)
	@rm -f $(installdir)/lib/$(libname).so

.PHONY : pkgfile
pkgfile:
	@echo
	@echo ------------------ creating pkg-config file
	@echo
	@echo "# Package Information for pkg-config"    >  $(pkgconfigfile)
	@echo "# Author= $(author)" 			>> $(pkgconfigfile)
	@echo "# Created= `date`"			>> $(pkgconfigfile)
	@echo "# Licence= $(licence)"			>> $(pkgconfigfile)
	@echo 						>> $(pkgconfigfile)
	@echo prefix=$(installdir)       		>> $(pkgconfigfile)
	@echo exec_prefix=$$\{prefix\}     		>> $(pkgconfigfile)
	@echo libdir=$$\{exec_prefix\}/lib 		>> $(pkgconfigfile)
	@echo includedir=$$\{prefix\}/include   	>> $(pkgconfigfile)
	@echo 						>> $(pkgconfigfile)
	@echo Name: "$(packagename)" 			>> $(pkgconfigfile)
	@echo Description: "$(description)" 		>> $(pkgconfigfile)
	@echo Version: "$(version)"                     >> $(pkgconfigfile)
	@echo Libs: -L$$\{libdir} -l$(packagename) ${custom_ld_flags}	>> $(pkgconfigfile)
	@echo Cflags: -I$$\{includedir\} ${define_flags} ${custom_cflags}  >> $(pkgconfigfile)
	@echo Requires: ${external_libraries}           >> $(pkgconfigfile)
	@echo Path=$(curpath)                           >> $(pkgconfigfile)
	@echo tagfile=$$\{Path\}/$(tag_file)            >> $(pkgconfigfile)
	@echo 						>> $(pkgconfigfile)

.PHONY : revert
revert :
	@mv -f makefile.in makefile

.PHONY : export
export :
	@echo "#automatically generated makefile"         >  $(automakefile)
	@echo packagename := ${packagename}               >> ${automakefile}
	@echo major_version := ${major_version}           >> ${automakefile}
	@echo minor_version := ${minor_version}           >> ${automakefile}
	@echo tiny_version  := ${tiny_version}            >> ${automakefile}
	@echo author := ${author}                         >> ${automakefile}
	@echo description := "${description}"             >> ${automakefile}
	@echo licence := ${licence}                       >> ${automakefile}
	@echo "#........................................" >> ${automakefile}
	@echo installdir := ${installdir}                 >> $(automakefile)
	@echo external_sources := ${external_sources}     >> ${automakefile}
	@echo external_libraries := ${external_libraries} >> ${automakefile}
	@echo libdir := ${libdir}                         >> ${automakefile}
	@echo srcdir := .                                 >> ${automakefile}
	@echo includedir:= ${includedir}                  >> ${automakefile}
	@echo define_flags := ${define_flags}             >> ${automakefile}
	@echo "#........................................" >> ${automakefile}
	@echo optimize := ${optimize}                     >> ${automakefile}
	@echo parallelize := ${parallelize}               >> ${automakefile}
	@echo f77 := ${f77}                               >> ${automakefile}
	@echo sse := ${sse}                               >> ${automakefile}
	@echo multi-threading := ${multi-threading}       >> ${automakefile}
	@echo profile := ${profile}                       >> ${automakefile}
	@echo "#........................................" >> ${automakefile}
	@echo specialize := ${specialize}                 >> ${automakefile}
	@echo platform := ${platform}                     >> ${automakefile}
	@echo "#........................................" >> ${automakefile}
	@echo sources := ${sources_list}                  >> ${automakefile}
	@echo                                             >> ${automakefile}
	@cat ${MAKEFILE_HEAVEN}/static-variables.makefile >> ${automakefile}
	@cat ${MAKEFILE_HEAVEN}/flags.makefile            >> ${automakefile}
	@cat ${MAKEFILE_HEAVEN}/rules.makefile            >> ${automakefile}
	@echo >> ${automakefile}
	@mv makefile makefile.in
	@mv ${automakefile} makefile

.PHONY : gflat
gflat :
	@echo "gprof $(exetarget) gmon.out -p | more"
	@gprof $(exetarget) gmon.out -p | more

.PHONY : gcall
gcall :
	@echo "gprof $(exetarget) gmon.out -q | more"
	@gprof $(exetarget) gmon.out -q | more

.PHONY : state
state  :
	@echo
	@echo "package name      : ${packagename} v${version} by ${author}"
	@echo "                   (${description}) "
	@echo "------------------------------------------------------------------------"
	@echo "install directory : ${installdir}"
	@echo "external sources  : ${external_sources}"
	@echo "external libs     : ${external_libraries}"
	@echo "fortran support   : ${f77}"
	@echo "------------------------------------------------------------------------"
	@echo "optimize          : ${optimize}"
	@echo "parallelize       : ${parallelize}"
	@echo "profile           : ${profile}"
	@echo "sse               : ${sse}"
	@echo "multi-threading   : ${multi-threading}"
	@echo "------------------------------------------------------------------------"
	@echo "specialize        : ${specialize} for ${platform}"
	@echo "------------------------------------------------------------------------"
	@echo "sources           : ${sources_list}"
	@echo "------------------------------------------------------------------------"
	@echo ldflags  = $(LDFLAGS)
	@echo cxxflags = $(CXXFLAGS)
	@echo sources = ${sources_list}
	@echo objects = ${objects}

.PHONY : rules
rules :
	@echo
	@echo ------------------ legitimate rules
	@echo
	@echo "(nothing)   : makes the executable : "
	@echo "library     : generates the library"
	@echo "slib        : generates shared library"
	@echo "install-slib: installs shared library"
	@echo "tags        : generates etags files"
	@echo "dox         : generates the doxygen documentation if Doxyfile exists"
	@echo "clear       : cleans up *~ #* and dependencies"
	@echo "clean       : cleans up .o lib and exe files"
	@echo "cleanaux    : cleans auxilary files: *.o *.d"
	@echo "cleandep    : cleans up the dependency files"
	@echo "cleandox    : cleans up the documentation"
	@echo "cleandist   : cleans everything except source+headers"
	@echo "install     : installs the executable"
	@echo "install-lib : installs the library"
	@echo "install-dev : installs the library along with documentation files"
	@echo "uninstall   : uninstalls the library"
	@echo "pkgfile     : generates the pkg-config file"
	@echo "flags       : shows the flags that will be used"
	@echo "gflat       : shows gprof profiler flat view result"
	@echo "gcall       : shows gprof profiler call graph view result"
	@echo "rules       : shows this text"
	@echo "state       : show the configuration state of the package"
	@echo "export      : export the makefile"
	@echo "revert      : moves makefile.in to makefile"
	@echo

${outdir}%.d : ${srcdir}%.c
	@echo
	@echo ------------------ creating dependencies for $@
	@echo
	$(compiler) $(CXXFLAGS) $(TARGET_ARCH) -MM $< | sed 's,\($(notdir $*)\.o\) *:,$(dir $@)\1 $@: ,' > $@.tmp
	mv -f $@.tmp $@
	@echo

${outdir}%.d : ${srcdir}%.cc
	@echo
	@echo ------------------ creating dependencies for $@
	@echo
	$(compiler) $(CXXFLAGS) $(TARGET_ARCH) -MM $< | sed 's,\($(notdir $*)\.o\) *:,$(dir $@)\1 $@: ,' > $@.tmp
	mv -f $@.tmp $@
	@echo

${outdir}%.d : ${srcdir}%.cpp
	@echo
	@echo ------------------ creating dependencies for $@
	@echo
	$(compiler) $(CXXFLAGS) $(TARGET_ARCH) -MM $< | sed 's,\($(notdir $*)\.o\) *:,$(dir $@)\1 $@: ,' > $@.tmp
	mv -f $@.tmp $@
	@echo

${outdir}%.o : ${srcdir}%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

${outdir}%.o : ${srcdir}%.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

${outdir}%.o : ${srcdir}%.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@


ifneq "$(MAKECMDGOALS)" "clean"
  include $(dependencies)
endif



