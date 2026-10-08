# Install built binaries to a running BMC over scp.
# Must be run from within a bitbake environment.
#
# Usage:
#   make install BMC=rain135bmc
#   make install BMC=rain135bmc:2222

RECIPE      := openpower-occ-control
REMOTE_USER := service
REMOTE_DIR  := /usr/local/share/hostfw/running
REMOTE_TMP  := /tmp/occ-patch
BINDIR      := $(BUILDDIR)/tmp/work/cortexa35-openbmc-linux/$(RECIPE)/1.0+git/image/usr/bin
BINARIES    := $(addprefix $(BINDIR)/,openpower-occ-control occctl)

install:
ifndef BMC
	$(error BMC is required.  Usage: make install BMC=<host>[:<port>])
endif
	$(eval _HOST := $(firstword $(subst :, ,$(BMC))))
	$(eval _PORT := $(or $(word 2,$(subst :, ,$(BMC))),22))
	$(eval _CTL  := /tmp/occ-ssh-$(_HOST).ctl)
	@ssh -p $(_PORT) -o ControlPath=$(_CTL) -O check $(REMOTE_USER)@$(_HOST) 2>/dev/null \
	    || ssh -p $(_PORT) -MNf -o ControlPath=$(_CTL) -o ControlPersist=120 $(REMOTE_USER)@$(_HOST)
	@echo "--> Preparing staging directory: $(REMOTE_TMP)"
	@ssh -p $(_PORT) -o ControlPath=$(_CTL) $(REMOTE_USER)@$(_HOST) "sudo bash -c 'mkdir -p $(REMOTE_TMP) && chmod 777 $(REMOTE_TMP)'"
	@echo "--> Copying from ./$(subst $(BUILDDIR)/,,$(BINDIR))/"
	@scp -O -P $(_PORT) -o ControlPath=$(_CTL) $(BINARIES) $(REMOTE_USER)@$(_HOST):$(REMOTE_TMP)/
	@echo "--> Moving files to $(REMOTE_DIR) ..."
	@ssh -p $(_PORT) -o ControlPath=$(_CTL) $(REMOTE_USER)@$(_HOST) "sudo bash -c 'cp $(REMOTE_TMP)/openpower-occ-control $(REMOTE_TMP)/occctl $(REMOTE_DIR)/'"
	@ssh -p $(_PORT) -o ControlPath=$(_CTL) -O exit $(REMOTE_USER)@$(_HOST) 2>/dev/null; true
	@echo "Install complete on $(_HOST):$(REMOTE_DIR)"

.PHONY: install
