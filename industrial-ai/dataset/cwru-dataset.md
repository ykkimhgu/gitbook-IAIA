# CWRU Dataset

### CWRU dataset

#### Introduction

**CWRU bearing dataset** https://engineering.case.edu/bearingdatacenter/download-data-file

**Classes:**

* Normal / Outer Race fault / Inner Race fault / Ball fault
* Different fault sizes
* Different load
* Drive-End, Fan-End

**Sampling:**

* Drive end bearing(12K): 12,000 samples/second
* Drive end bearing(48K): 48,000 samples/second (not used in lecture)
* Fan end bearing data: 12,000 samples/second. (not used in lecture)

**Variable names**

* DE - drive end accelerometer data
* FE - fan end accelerometer data
* BA - base accelerometer data
* time - time series data
* RPM - rpm during testing

![](https://github.com/user-attachments/assets/35099f50-ca84-42bd-8219-128680bee37a)

#### Dataset folders

**CWRU\_selected\_dataset**

* Feature\_data
  * env\_feature
  * stat\_feature
  * wpe\_feature
  * glob\_feature
  * example\_data --> sample\_data
  * example\_test --> sample\_test
  * example\_train --> sample\_train
* Raw\_data
  * ball\_007\_1hp
  * inner\_007\_1hp
  * normal\_1hp
  * outer\_007\_1hp

**CWRU\_full\_dataset**

* Raw\_data\_0hp
  * 12k Drive-End(DE), Fan-End(FE)
  * Normal, Ball, IR, OR
  * Fault size: 7, 14, 21
  * Load: 0HP
*   Raw\_data

    * 12k Drive-End(DE), Fan-End(FE)
    * Normal, Ball, IR, OR
    * Fault size: 7, 14, 21
    * Load: 0HP, 1hp, 3hp

    ![](https://github.com/user-attachments/assets/7addc9d8-e6ae-4de2-94cf-f594e2dd6c32)
