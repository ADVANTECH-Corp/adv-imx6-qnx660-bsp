TYPE = $(firstword $(filter a o dll, $(VARIANTS)) o)
BOARD = $(firstword $(filter mpc512x mx25ads mx27ads mx6sabrelite tegra3 armada610 msm8960 pxa978, $(VARIANTS)))
USEFILE_dll = $(SECTION_ROOT)/$(BOARD)/$(PROTOCOL)/$(NAME)-$(PROTOCOL)-$(BOARD).use
USEFILE   = $(USEFILE_$(TYPE))
