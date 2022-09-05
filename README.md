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
{"set":"cal","to":[9.1,9.2,9.3,9.4,9.5,9.6],"auth":"bar"}  // should be ignored with {"error":"wrong secret"}
{"set":"cal","to":[1.0,1.1,1.2,1.3,1.4,1.5,1.6],"auth":"foo"}  //should work
{"set":"cal","to":[1,2,3,4],"auth":"foo"} // should fail with {"error":"wrong number of values in cal array","want":7,"have":4}
{"get":"cal"} // should give last good cal details
{"set":"cal","to":[2.0,2.1,2.2,2.3,2.4,2.5,2.6],"auth":"foo"}  //should work
{"set":"cal","to":[3.0,3.1,3.2,3.3,3.4,3.5,3.6],"auth":"foo"}  // since MAX_FLASH_WRITES is set to 3, this fourth write won't work
{"get":"cal"} // should give last good cal details
```

A successful secret setting returns
```
{"log":"secret","is":"set"}
```

A successful cal update returns something in this format: 
```
{"log":"cal","is":"ok","values":[0.00,0.10,0.20,0.30,0.40,0.50,0.60],"writes_remaining":2}
```


You unplug the arduino, and plug it in again, and the calibration data is retained.


### Production

There's no need to report the cal routinely, there is a command to get the current calibration state so you can check what values are in use.

A sensible write limit is probably 20, which is probably about 5x more than we'd need for setting up a new experiment, even allowing for a few go-arounds.

