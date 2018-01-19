# -----------------------------------------------------------------------------------
# DESCRIPTION:
# This file names preprocessed embedded files for use by all projects.
# Typically these include .js and .html files.
# Conditional inclusion typically looks like:
#
# $(if $(filter MY_FLAG, $(FEATURE_FLAGS)), \
#     $(BASE_DIRECTORY)/module/my_module/my.js \
# ) \
#
# ALERT:
# Files added to these variables should maintain the alphabetical order.
# -----------------------------------------------------------------------------------

PREPROC_EMBEDDED := \
  $(BASE_DIRECTORY)/module/custom_data/http/help/config.Custom.html \
  $(BASE_DIRECTORY)/module/custom_data/http/web/user_data_specification.js \
