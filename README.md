# GeoISA
The main part of the source code for the GeoISA protocol for content discovery in VNDN and the related selected benchmarked solutions.

<br /> In this public repository, we'd like to share the source code for the new protocol for content discovery in named data vehicular networks, which is called GeoISA.

<br /> We also share the source code of the previous works that were selected as benchmarks to compare the performance of the GeoISA protocol. They are  as follows:

1. Basic NDN-based Flooding (BNF) scheme 
  (reference: L. Zhang, A. Afanasyev, J. Burke, V. Jacobson, K. Claffy, P. Crowley, C. Papadopoulos, L. Wang, B. Zhang, Named data networking, ACM SIGCOMM    Computer Communication Review. 44 (2014) 66–73. https://doi.org/10.1145/2656877.2656887.)
2. Vanilla VNDN 
  (reference: G. Grassi, D. Pesavento, G. Pau, R. Vuyyuru, R. Wakikawa, L. Zhang, VANET via Named Data Networking, in: 2014 IEEE Conference on Computer        Communications Workshops (INFOCOM WKSHPS), IEEE, 2014: pp. 410–415. https://doi.org/10.1109/INFCOMW.2014.6849267.)
3. CA-VNDN scheme 
  (reference: Y. Li, X. Shi, A. Lindgren, Z. Hu, P. Zhang, D. Jin, Y. Zhou, Context-Aware Data Dissemination for ICN-Based Vehicular Ad Hoc                    Networks,Information. 9 (2018) 263. https://doi.org/10.3390/info9110263.)
4. Sweet forwarding spot (SFS) scheme 
  (reference 1: 
   L.B. Rondon, J.B.D. da Costa, G.P.R. Filho, L.A. Villas, A Distance and Position-based Caching Discovery Protocol for Vehicular Named-Data Networks, in:    2019 IEEE Latin-American Conference on Communications (LATINCOM), IEEE, 2019: pp. 1–6. https://doi.org/10.1109/LATINCOM48065.2019.8938022.,
   reference 2:W.U.I. Zafar, M.A.U. Rehman, F. Jabeen, B.-S. Kim, Z. Rehman, Context-Aware Naming and  Forwarding in NDN-Based VANETs, Sensors. 21 (2021)      4629. https://doi.org/10.3390/s21144629.)
   
The <a href="mailto:p92824@siswa.ukm.edu.my">developers</a> of GeoISA are open to inquiries about what is in this repository as further details of the code, or any future research work.

To whom it may find the source code published in this repository is useful to their reseach work, please indicate to that in your published research work by referring to the following reference:
The data will be updated later.


<b>How to use</b>

<b>Requirement</b>
1. ndnSIM 2.8 , to install ndnSIM click <a href="https://ndnsim.net/2.8/getting-started.html">here</a>
2. SUMO 1.8.0, to install SUMO click <a href="https://sumo.dlr.de/docs/Downloads.php">here</a>

<b> To get access to the source code folder, click <a href="https://drive.google.com/drive/folders/1dr6qWBoaU3-99eqwSErud6pmVP06UPze?usp=sharing">source code</a> </b> 
<br /> In the source code Folder, there are six folders. Each folder contains the source code for a specific forwarding protocol. To make the code ready to run, follow the following: (<b>Note: </b><u><i>replace any old files/folders if needed</i></u>)
 1. Install ndnSIM and make sure it works properly.
 2. <b>Copy</b> the content in <b>scratch folder</b> in the <b>source code folder</b> to <b>scratch folder</b> in <b>ns-3 folder</b> in your local computer.
 3. <b>Copy</b> the content in <b>ns3 folder</b> in the <b>source code folder</b> <b><u>directly</u></b> to <b>ns-3 folder</b> in your local computer.
 4. <b>Copy</b> <b>ndnSIM folder</b> in the s<b>source code folder</b> <b><u>directly</u></b> to <b>src folder</b>  in <b>ns-3 folder</b> in your local computer</b>.
  
<b>BNF Folder</b> 
<br /> This folder contains the main files to run the BNF algorithm.

<b>SUMO Folder</b> 
<br /> This folder contains the sumo and TCL files used in the simulation.

<b>Vanilla VNDN Folder</b> 
<br /> This folder contains the main files to run the Vanilla VNDN algorithm.
  
<b>SFS Folder</b> 
<br /> This folder contains the main files to run the SFS algorithm.

<b>CA-VNDN 45 Folder</b>
<br /> This folder contains the main files to run the CA-VNDN algorithm, where forwarding suppression angle is 45.
 
<b>CA-VNDN 225 Folder</b>
<br /> This folder contains the main files to run the CA-VNDN algorithm, where forwarding suppression angle is 22.5.

<b>GeoISA Folder</b> 
<br /> This folder contains the main files to run the GeoISA algorithm. 
<br /> GeoISA uses TraCI tool (https://sumo.dlr.de/docs/TraCI.html) to make online interaction between network(NS-3/ndnSIM) and road traffic (SUMO). However, in the current published GeoISA, TraCI is not used. GeoISA with TraCI while be publish later.So, we use roads of same number of lanes.
