# nanoflash
Flashstorage demo for the nano IOT 33 - this might be useful for storing remote lab experiment calibration data 

## Proposed approach

We want to store calibration data on a nano IOT 33. It is proposed to provide two commands:

- set the authorisation string and maximum number of writes allowed 
  - update the stored data

Other protections include:

- Enforce a maximum of 100 permissable writes by default, as back up protection against chip damage in the case that a sysadmin accidentally makes a system accessible to users before setting the authorisation code. Do not permit the number of writes to be set higher than 100, only lower.


Note that writing new firmware to the nano IOT uses the Flash storage, so a solution that modified the firmware source code and required a recompile does not offer any lifetime preservation over this approach, and is probably worse (rewrites more of the memory).

## Requirements

Flash storage supports approx 10k writes over the lifetime, so unwanted writes are able to damage the chip eventually. Therefore we require to 

- protect from over usage 
- protect from unauthorised usage

### Over-usage

There are limited write-cycles available for Flash memory - ca 10k. A looped command to write data to flash can soon exceed this limit (10 writes per second would exceed this limit in less than a half hour). Possible strategies

- bursted rate limiting (allow a few writes during an admin session, but no more than a few an hour)
- limit maximum number of updates per firmware flash 

Writing a suitable burst-permitting rate limiting algorithm is probably overkill, whereas preventing more than a certain number of writes per firmware flash is probably functionally equivalent if, and only if, only authorised users can trigger writes.

Forcing the firmware to be reflashed after a certain number of writes prevents a malicious user  from damaging the chip unless they've also managed to gain control over the host single board computer (in which case, they can write any firmware they like). Eventually requiring reflashing is not inconvenient when you consider most calibrations will be done one for the initial install, and then after any physical maintenance. A few calibrations a year is probably typical, so needing more than a 100 in a given year is probably an indication the experiment needs EEPROM added. Hitting 100 calibrations is (a) unlikely, (b) would typically occur when the system is under maintenance with good access to the single board computer for the sysadmin to do any said reflashing.


### Authorised usage

Authorisation implies checking incoming commands meet some sort of criteria, but doing so without access to the usual authorisation features in our cloud infrastructure. Some requirements include

- no credentials in the firmware repo
- no pre-build code insertion in the firware (complicates the ansible playbooks)

An alternative strategy is to permit an initial usage of the write command, which should/must set the credential, and potentially the maximum number of writes. This can be issued by the sysadmin before making the system live.


### Commands

```
{"set":"secret","to":"foo"}   //should work
{"set":"secret","to":"bar"}   // should be ignored
{"set":"cal","to":[0.0,0.1,0.2,0.3,0.4,0.5,0.6],"auth":"foo"}  //should work
{"set":"cal","to":[9.1,9.2,9.3,9.4,9.5,9.6],"auth":"bar"}  // should be ignored with {"error":"wrong secret","want":"foo","have":"bar","warning":"you are revealing secrets - do not release this code into production"}
{"set":"cal","to":[1,2,3,4],"auth":"foo"} // should fail with {"error":"wrong number of values in cal array","want":7,"have":4}

```


