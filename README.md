# 3D Cell-Free / Distributed MIMO Channel Estimation Simulator

## 主要変数のdimension

| 変数 | dimension | 意味 |
|---|---:|---|
| `APpos` | `M x 3` | AP中心の `[x,y,z]` |
| `UEpos` | `K x 3` | UEの `[x,y,z]` |
| `antennaPos` | `M x N_AP x 3` | AP各アンテナ素子の実座標 |
| `p` | `tau_p x K` | unit-norm uplink pilot |
| `h_true` | `N_AP x M x K` | geometryから作るground-truth channel |
| `Y(:,:,m)` | `N_AP x tau_p` | AP mが実際に観測するpilot matrix |
| `y_mk` | `N_AP x M x K` | `Y(:,:,m)*p(:,k)` |
| `h_hat_LS` | `N_AP x M x K` | APのpilot観測から得たLS推定値 |
| `NMSE` | `M x K` | link-wise NMSE |

コードの主要境界にはdimensionを確認する`assert`を置いている。

## 1. 研究目的と処理フロー

対象は次の範囲だけである。

`3D AP/UE/建物配置 -> 3D LoS/単反射Ray Tracing -> h_true -> uplink pilot -> AP受信Y -> despreading -> h_hat_LS -> NMSE`

`h_true`はシミュレータだけが知るground truthであり、APに直接与えない。AP側推定器が利用するのは`Y(:,:,m)`、既知pilot、既知pilot energyだけである。GE-MCTS、MCTS、Graphormer、user selection/admission、SDR、AP/downlink power allocation、downlink beamforming、SINR最適化、UE数最大化は実装していない。

Ray Tracing moduleとchannel estimator moduleは独立している。前者の出力`h_true`は`simulatePilotReception`を介してのみ後者の観測へ入る。

## 2. 参考にした2論文と境界

### 論文1

H. Zhou, X. Liu, and S. Lambotharan, "A General Sensing-Assisted Channel Estimation Framework in Distributed MIMO Network," IEEE Wireless Communications Letters, 2025.

そのまま参考にした概念:

- Distributed MIMO、LoS/NLoS、複数pathのcoherent sum
- single-bounce reflection、鏡像UEと反射面の交点、path existence
- path length、phase、AoA/AoD、array response、複素反射特性
- paper reference配置: AP `(0,0)`, `(200,200)`, `(0,200)`, `(200,0)`, `(100,200)`、UE `(50,150)`, `(150,150)`, `(150,100)`

今回のExtension:

- 論文の移動sensing target位置推定ではなく、シミュレータ既知の3D building ground truth
- 複数のyaw付き3D rectangular cuboid、全4側面とroofの探索
- 3D segment-cuboid intersection、AP/UE高、elevation、アンテナ実座標
- 論文式(19)/(20)の2D有効開口・specular/diffuse係数を移植せず、Friis field amplitudeと複素`Gamma`を使うpaper-inspired simplified reflection model

したがって、本コードは論文1の完全再現でも、同論文のsensing-assisted estimatorでもない。

### 論文2

A. P. Singh, A. Chowdhury, and R. Chopra, "Pilot Clustering in Cell-Free mMIMO Systems Under Mixed LoS/NLoS Channels," NCC 2026.

そのまま参考にした概念:

- unit-norm orthogonal pilot、pilot reuse集合、pilot contamination、AWGN
- 式(5)のsample-wise received pilot
- 式(6)のdespread後の`desired + contamination + noise`
- 式(7)のMMSE coefficientとMSE
- simulation reference値として`fc=3 GHz`、AP高`10 m`、UE高`1.5 m`

そのまま組み合わせない部分:

- 論文2は`h_mk = delta_mk*hbar_mk + sqrt(beta_mk)*hdot_mk`、`hdot_mk ~ CN(0,I)`、既知`beta_mk`を仮定する。
- 論文の`c_mk`は、この確率分布と白色雑音から得る。1個の決定論的Ray-Tracing realizationには`E[h]`と`Cov[h]`が自動的には存在しない。
- よってHybridの`h_true`へ論文の`c_mk`をコピーしない。論文式(5)-(7)は`channelModel="pilotPaperStochastic"`で独立検証する。
- Hybridの`pilotUserMode="allUE"`は全AP-UE linkの推定値を得るためのExtensionで、論文2のNLoS-only trainingとは異なる。
- `clusteredReuse`は距離重み付きgreedy coloringであり、論文のstochastic-geometry clusteringの厳密再現ではないExtensionである。

## 3. 3D座標系とscenario

Cartesian座標`[x,y,z]`を使う。default領域は`x=0..200 m`, `y=0..200 m`, `z=0..30 m`。defaultは`M=5`, `K=10`, `N_AP=4`, AP高`10 m`, UE高`1.5 m`, `fc=3 GHz`, `lambda=c/fc`である。

- `scenarioMode="random3D"`: AP/UE/複数建物をseed付きで生成する研究用Extension。
- `scenarioMode="rayTracingPaperReference"`: 論文1の2D配置を正確に用い、高さだけconfigから付加する。論文にないrandom buildingは混ぜない。

APはconfigurable directionのULAで、素子間隔はdefault`lambda/2`。素子nの位相は中心からの実座標差`r_n-r_AP`とpath direction`u`から

`a_n(u) = exp(-j*2*pi/lambda * dot(r_n-r_AP,u))`

として計算する。

## 4. 遮蔽物、LoS判定、単反射、knife-edge回折

各buildingは`center, width, depth, height, yaw, reflectionCoefficient`を持つ。random rangeはwidth/depth `10..40 m`、height `5..25 m`。AP/UE内部配置と、保守的bounding circleが重なるbuilding配置を拒否し、最大試行回数で停止する。

LoS判定はglobal点をbuilding local frameへ回転し、slab methodで有限3D線分とaxis-aligned boxの交差を調べる。したがって、XY投影がbuildingと重なってもpathのzがroofより高ければ遮断しない。

単反射は各4側面およびroofに対して次を行う。

1. UEを無限planeへmirrorする。
2. AP-mirror UE線とplaneを交差させる。
3. 交点が有限rectangle内か判定する。
4. AP-reflection pointとreflection point-UEの両segmentが他のcuboidに遮られないことを確認する。
5. `Gamma * lambda/(4*pi*D) * exp(-j*2*pi*D/lambda)`とarray responseを計算する。

`Gamma`のrandom magnitude/phase、Friis field amplitude、side/roof reflection選択はSimulation Assumption / Extensionであり、論文1の式(20)そのものではない。

LoSがcuboidに遮断されたlinkでは、各blocking cuboidのroof perimeter 4辺とvertical 4辺をknife-edge候補とする。各有限edge上で`AP-edge-UE`総距離を最小にする点を探索し、他のcuboidに遮られない候補のうち回折損失が最小の支配的pathを各blocking obstacleにつき最大1本採用する。回折パラメータは

`v = h*sqrt(2*(d1+d2)/(lambda*d1*d2))`

とし、`v>-0.78`では[ITU-R P.526-16](https://www.itu.int/rec/R-REC-P.526-16-202511-I/en)のsingle knife-edge近似

`J(v) = 6.9 + 20*log10(sqrt((v-0.1)^2+1)+v-0.1) [dB]`

を使用する。`h`は候補edge点からAP-UE直線への垂直距離、`d1`,`d2`はAP-edge点、edge点-UE距離である。回折振幅係数は`10^(-J(v)/20)`、位相はbroken path lengthに対する`exp(-j*2*pi*D/lambda)`で与える。

重要なSimulation Extension/Assumption:

- 有限厚cuboidを回折点近傍でzero-thickness knife edgeに置き換える。
- ITU近似が与えるmagnitude lossを使用し、追加のFresnel diffraction phaseは無視する。
- 厳密なUTD wedge coefficient、偏波、導電率、複素誘電率、表面粗さは未導入。
- defaultは`enableDiffraction=true`。無効化する場合は`false`とする。

AoA/AoDのbearingは、direction`u=[u_x,u_y,u_z]`に対して

- `azimuth = atan2(u_y,u_x)`
- `elevation = atan2(u_z,sqrt(u_x^2+u_y^2))`

と定義する。LoS、全有効1回反射、全有効1回knife-edge回折をcomplex phaseを保持して足し、`h_true(:,m,k)=sum_l h_path(:,l)`を得る。

## 5. Pilot、AP受信、contamination

DFT basisから`||p_k||^2=1`のpilotを作る。

- `orthogonal`: `tau_p>=K`、異なるUEのinner productは0。
- `randomReuse`: `tau_p<K`を想定してpilotをrandom reuse。
- `clusteredReuse`: 近いUEほど同じ色を避けるgreedy Extension。

AP mの観測は

`Y_m = sum_k sqrt(E_p(k))*h_true(:,m,k)*p_k^H + W_m`

で、`W_m`の各elementは`CN(0,sigma2)`。despread後は

`y_mk = Y_m*p_k`

`= sqrt(E_p(k))*h_true_mk + sum_{j!=k} sqrt(E_p(j))*h_true_mj*(p_j^H p_k) + W_m*p_k`

となる。同じpilotをreuseするUEの項がpilot contaminationである。

Hybridのdefault noise relationは、`P_ref = mean_{m,k}(||h_true(:,m,k)||^2/N_AP)`として

`sigma2 = mean(E_p)*P_ref / 10^(pilotSNRdB/10)`

である。すなわち`pilotSNRdB`はnetwork-average received channel power基準。`noiseMode="fixed"`では`noiseVarianceFixed`を直接使える。遮蔽物ON/OFFのmain比較では、OFF側で決めた同じ`sigma2`と同じrandom streamを両方へ使用する。

## 6. LSとoptional LMMSE

unit-norm pilotなのでHybridの正式推定器は

`h_hat_LS(:,m,k) = Y(:,:,m)*p(:,k)/sqrt(E_p(k))`

である。priorを仮定せず、noise=0、pilot contaminationなしなら`h_hat_LS=h_true`となる。

`estimateChannelLMMSE.m`には、外部から明示的な`mu_mk`と`R_mk`を渡した場合だけ使える一般式を実装した。ただしdefault実行ではpriorを捏造せず、`h_hat_LMMSE=[]`である。将来Monte-Carlo ensembleをtraining/evaluationへ分離し、平均・共分散を学習する場合にのみ有効化する。

## 7. 論文2の独立validation mode

`validatePilotPaperModel`はRay-Tracingを一切使わず、`hdot_mk~CN(0,I)`と`beta_mk`を生成する。式(5)の`sqrt(tau_p*beta_mk*E_p,k)`、式(6)のpilot相関、式(7)の`c_mk`とMSEを再現し、empirical MSEが式(7)へ収束することを確認する。

単独実行例:

```matlab
cfg = configChannelSim3D('channelModel','pilotPaperStochastic');
validation = main_channel_estimation_3D(cfg);
```

このmodeの`c_mk`をRayTracingHybridへ流用してはならない。

## 8. NMSE

各linkについて

有効な伝搬pathを持つlinkについて

`NMSE_mk = ||h_hat(:,m,k)-h_true(:,m,k)||^2 / ||h_true(:,m,k)||^2`

を計算する。`||h_true||^2 <= epsilon`のlinkは、受信雑音をepsilonで割ると平均NMSEを物理的に無意味な値へ膨張させるため、channel-estimation errorではなく伝搬outageとして`outageMask`へ分離する。このlinkの`NMSE`は`NaN`とする。全有効linkの線形NMSEを平均して`meanNMSE`とし、表示時に`10*log10`を使う。`outageFraction`は全AP-UE linkに占めるoutage率である。

## 9. Figureと保存結果

`main_channel_estimation_3D`は次を表示・保存する。

1. 3D AP/UE/cuboid/selected LoS-reflection path
2. selected linkの`|h_true|`/`|h_hat_LS|`とphase
3. pilot SNR vs Monte-Carlo mean NMSE: orthogonal/random/clustered reuse
4. 同じAP/UE、共通雑音によるobstacle OFF/ON
5. `10*log10(||h_true||^2)`のM x K heatmap
6. link NMSEのM x K heatmap

MAT-fileにはscenario、AP/UE/obstacle/antenna位置、`h_true`、`h_hat_LS`、空の`h_hat_LMMSE`、pilot、assignment、SNR、noise、`Y`、`y_mk`、NMSE、pathInfo、Monte-Carlo結果、paper validation結果を保存する。`pathInfo(m,k).paths`にはtype、length、reflection point、AoA/AoD、complex gain、per-path antenna contributionなどが入る。

`inspectLink(result,m,k)`は、選択linkの位置、LoS block、reflection、path length/gain、`h_true`、`y_mk`、`h_hat_LS`、NMSEを表示し、geometryとchannel比較を再描画する。

## 10. 実行方法

MATLABでこのfolderをcurrent folderにして実行する。

```matlab
runSanityChecks
result = main_channel_estimation_3D;
```

短いdebug run:

```matlab
cfg = configChannelSim3D(struct( ...
    'numRealizations',5, ...
    'pilotSNRdB',[0 10], ...
    'showFigures',true));
result = main_channel_estimation_3D(cfg);
```

paper reference geometry:

```matlab
cfg = configChannelSim3D('scenarioMode','rayTracingPaperReference', ...
    'tauP',3,'pilotAssignmentMode','orthogonal');
result = main_channel_estimation_3D(cfg);
```

default Monte-Carloは100 realizations。`numRealizations=1000`へ変更可能。`rng(seed,'twister')`を使用し、同一seedでscenario、channel、pilot、noise、estimateを再現する。

100 GHzで実行する場合は`fc=100e9`とする。`arraySpacing`を明示指定しなければ、波長`3 mm`に対応する半波長間隔`1.5 mm`へ自動更新される。

```matlab
cfg = configChannelSim3D(struct('fc',100e9,'numRealizations',100));
result = main_channel_estimation_3D(cfg);
```

defaultの`networkAverageReceivedSNR`は各周波数の平均受信チャネル電力を基準に雑音を設定するため、同じ実効Pilot SNRで推定方式を比較する設定である。3 GHzと100 GHzの絶対的なlink budget差を比較する場合は、両周波数で`noiseMode="fixed"`、同一`pilotEnergy`、同一`noiseVarianceFixed`を使用する必要がある。

## 11. Phase 1-10とsanity checks

`runSanityChecks.m`は次の順で停止判定する。

| Phase | 自動確認 |
|---:|---|
| 1 | Test 1: 1 AP/1 UE/no obstacle/no noiseでLS relative errorがほぼ0 |
| 2 | 4-element ULAの予測phase progression |
| 3 | Test 3: multi-UE orthogonal pilotで他UE項が消える |
| 4 | Test 4: reuseでcontaminationが非zero |
| 5 | Test 5/6: low building blockとover-roof LoSの3D差 |
| 6 | Test 7/8: `Gamma=0`反射zero、blocked LoSでもreflection channel非zero |
| 7 | Test 9/10: obstacle OFF LoS、同一seed再現 |
| 8 | Test 2: Monte-Carlo平均NMSEがSNR上昇で低下 |
| 9 | clustered reuseで近接UE pairを異なるpilotへ割当 |
| 10 | 論文2式(5)-(7)のpredicted/empirical MSE一致 |

## 12. MATLAB Toolbox

必須はbase MATLABだけである。Antenna Toolbox、Communications Toolbox、Phased Array System Toolbox、RF Propagation/raytraceは使用しない。`reportToolboxes`は現在のinstallationを表示する。全ての3D intersection、image method、array response、pilot、AWGNはコード内で実装している。

## 13. MATLABファイル一覧

- entry/config: `main_channel_estimation_3D`, `configChannelSim3D`, `reportToolboxes`
- scenario: `generateScenario3D`, `generateAPPositions3D`, `generateUEPositions3D`, `generateObstacles3D`, `generateAPArrayPositions`
- geometry/Ray Tracing: `generateRayTracingChannels3D`, `checkLoS3D`, `isBlocked3D`, `segmentIntersectsCuboid`, `transformToObstacleLocal`, `findSingleBounceReflections3D`, `findSingleEdgeDiffractions3D`, `computeKnifeEdgeDiffractionLoss`, `mirrorPointAcrossPlane`, `linePlaneIntersection`, `pointInsideRectangle3D`, `computePathGain`, `computeAoAAoD3D`, `computeArrayResponse3D`
- pilot/estimation: `generatePilots`, `assignPilots`, `selectPilotUsers`, `simulatePilotReception`, `despreadPilot`, `estimateChannelLS`, `estimateChannelLMMSE`, `computeNoiseVariance`, `computeNMSE`
- evaluation: `runSingleSimulation3D`, `runMonteCarlo3D`, `validatePilotPaperModel`, `runSanityChecks`, `inspectLink`
- plotting: `plotScenario3D`, `plotChannelComparison`, `plotNMSEvsSNR`, `plotObstacleComparison`, `plotChannelGainHeatmap`, `plotNMSEHeatmap`

## 14. 既知の制限事項

- LoS、single-bounce specular reflection、簡略single knife-edge diffractionを扱う。厳密UTD、複数edge回折、反射後の回折、回折後の反射、diffuse scattering、多重反射、ground reflectionは未実装。
- building materialの周波数依存Fresnel係数は未導入。random complex`Gamma`はAssumption。
- cuboid overlap回避はconservative bounding circleのため、置ける建物密度を過小評価する場合がある。
- narrowband flat-fading complex channel。delay tap/OFDM、Doppler、synchronization error、channel agingは扱わない。
- AP array bearingはphysical-coordinate phaseで表現するが、element patternとmutual couplingは扱わない。
- `paperNLoSOnly`ではLoSを1本でも持つUEをtraining対象外にする簡易UE-level判定。論文の全stochastic geometryをHybridへ持ち込むものではない。
- Hybrid LMMSEは正当な独立training ensembleがないためdefault無効。これは意図的な設計である。
