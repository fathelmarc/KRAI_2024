# KRAI_2024
update tanggal 9 Nov
robot dapat bergerak dan mengambil seedling dengan control remote.
perbaikan:
1. buat tempat untuk peletakkan seedling
2. perbaiki sensor untuk putaran gripper
   a. untuk saat ini opsi yang dipunya adalah menggunakan wormwindow atau solenoid untuk stopper mekanik,
   dan sensor tambahan untuk indikasi gripper untuk menjepit opsi saat ini menggunakan limit yang batangnya menutup 2 arah
   sensor IR (kuning), atau bisa menggunakan optik.
3. untuk pengoperasian robot akan digunakan auto jika memungkinkan dan jika tidak maka akan dilakukan semi-auto.
   opsi pertama adalah menggunakan delay dikarenakan gerakannya hanya maju kekanan dan mundur, tapi ada masalah pada keakuratan.
   jika tanpa menggunakan delay opsi yang terpikirkan adalah menggunakan ros serial atau ethernet untuk melakukan penghitungan encoder pada pc/laptop.
   berikut kendala yang mungkin dihadapi.
   a. menggunakan ros serial(kemungkinan pembelajarannya lama)
   b. ethernet perakitan rangkaian elektrikal mungkin makan waktu serta belum bisa dipastikan pengirimannya aman atau tidak.
4. 
