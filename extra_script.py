Import("env")

# to build date str
import datetime
date = datetime.datetime.now()
date = date.strftime("%y") + date.strftime("%m") + date.strftime("%d")

# name from env:project
build_name = env['PIOENV']
build_version = "422dev" + date

env.Replace(PROGNAME="%s" % build_name + "_" + "%s" % build_version)
