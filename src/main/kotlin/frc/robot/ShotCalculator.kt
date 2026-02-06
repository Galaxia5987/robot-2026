package frc.robot

import kotlin.math.pow

fun calculateVelocity(distance: Double, vx: Double, vy: Double): Double =
    3.38893343 +
        (1.14129528) * distance +
        (0.10395370) * vx +
        (0.07514133) * vy +
        (-0.08841128) * distance.pow(2) +
        (-0.31626913) * distance * vx +
        (-0.02619092) * distance * vy +
        (0.18777928) * vx.pow(2) +
        (-0.00138487) * vx * vy +
        (0.09627630) * vy.pow(2) +
        (0.01090175) * distance.pow(3) +
        (0.04107728) * distance.pow(2) * vx +
        (0.00440369) * distance.pow(2) * vy +
        (-0.03453064) * distance * vx.pow(2) +
        (0.00028481) * distance * vx * vy +
        (-0.00654656) * distance * vy.pow(2) +
        (-0.00126298) * vx.pow(3) +
        (-0.00119483) * vx.pow(2) * vy +
        (-0.02390827) * vx * vy.pow(2) +
        (0.00047597) * vy.pow(3) +
        (-0.00059611) * distance.pow(4) +
        (-0.00202557) * distance.pow(3) * vx +
        (-0.00027602) * distance.pow(3) * vy +
        (0.00235936) * distance.pow(2) * vx.pow(2) +
        (0.00000634) * distance.pow(2) * vx * vy +
        (0.00048720) * distance.pow(2) * vy.pow(2) +
        (0.00021067) * distance * vx.pow(3) +
        (0.00015657) * distance * vx.pow(2) * vy +
        (0.00318419) * distance * vx * vy.pow(2) +
        (-0.00004623) * distance * vy.pow(3) +
        (-0.00037451) * vx.pow(4) +
        (-0.00003362) * vx.pow(3) * vy +
        (-0.00134212) * vx.pow(2) * vy.pow(2) +
        (0.00013003) * vx * vy.pow(3) +
        (0.00102131) * vy.pow(4)

fun calculatePitch(distance: Double, vx: Double, vy: Double): Double =
    54.00000000

fun calculateYaw(distance: Double, vx: Double, vy: Double): Double =
    -15.97452577 +
        (8.93053556) * distance +
        (-0.60869908) * vx +
        (-21.50632771) * vy +
        (-1.56563587) * distance.pow(2) +
        (-7.22714897) * distance * vx +
        (2.64099688) * distance * vy +
        (-4.05387301) * vx.pow(2) +
        (7.72263666) * vx * vy +
        (0.47663796) * vy.pow(2) +
        (-0.25616593) * distance.pow(3) +
        (7.73148070) * distance.pow(2) * vx +
        (-0.93193941) * distance.pow(2) * vy +
        (2.23595092) * distance * vx.pow(2) +
        (-8.47045460) * distance * vx * vy +
        (-0.53759788) * distance * vy.pow(2) +
        (-4.03174346) * vx.pow(3) +
        (4.23189624) * vx.pow(2) * vy +
        (-0.59657072) * vx * vy.pow(2) +
        (-1.19343078) * vy.pow(3) +
        (0.13291045) * distance.pow(4) +
        (-3.48733586) * distance.pow(3) * vx +
        (0.59524908) * distance.pow(3) * vy +
        (0.13080251) * distance.pow(2) * vx.pow(2) +
        (3.16920030) * distance.pow(2) * vx * vy +
        (0.31605693) * distance.pow(2) * vy.pow(2) +
        (3.45485751) * distance * vx.pow(3) +
        (-3.23518717) * distance * vx.pow(2) * vy +
        (0.50338280) * distance * vx * vy.pow(2) +
        (0.94324802) * distance * vy.pow(3) +
        (-1.23458144) * vx.pow(4) +
        (0.67182134) * vx.pow(3) * vy +
        (-0.18461704) * vx.pow(2) * vy.pow(2) +
        (-0.92982486) * vx * vy.pow(3) +
        (0.04967652) * vy.pow(4) +
        (-0.01273087) * distance.pow(5) +
        (0.84773645) * distance.pow(4) * vx +
        (-0.19158847) * distance.pow(4) * vy +
        (-0.35803593) * distance.pow(3) * vx.pow(2) +
        (-0.59945492) * distance.pow(3) * vx * vy +
        (-0.10320673) * distance.pow(3) * vy.pow(2) +
        (-1.14541663) * distance.pow(2) * vx.pow(3) +
        (1.01368933) * distance.pow(2) * vx.pow(2) * vy +
        (-0.17460809) * distance.pow(2) * vx * vy.pow(2) +
        (-0.29146015) * distance.pow(2) * vy.pow(3) +
        (0.91106746) * distance * vx.pow(4) +
        (-0.43649648) * distance * vx.pow(3) * vy +
        (0.12518554) * distance * vx.pow(2) * vy.pow(2) +
        (0.58234357) * distance * vx * vy.pow(3) +
        (-0.03080926) * distance * vy.pow(4) +
        (-0.16535574) * vx.pow(5) +
        (0.04816541) * vx.pow(4) * vy +
        (0.00004039) * vx.pow(3) * vy.pow(2) +
        (-0.16704486) * vx.pow(2) * vy.pow(3) +
        (0.02901435) * vx * vy.pow(4) +
        (0.03872888) * vy.pow(5) +
        (-0.00139849) * distance.pow(6) +
        (-0.11630965) * distance.pow(5) * vx +
        (0.03181943) * distance.pow(5) * vy +
        (0.10062490) * distance.pow(4) * vx.pow(2) +
        (0.05993158) * distance.pow(4) * vx * vy +
        (0.01889450) * distance.pow(4) * vy.pow(2) +
        (0.18136681) * distance.pow(3) * vx.pow(3) +
        (-0.16125388) * distance.pow(3) * vx.pow(2) * vy +
        (0.03110772) * distance.pow(3) * vx * vy.pow(2) +
        (0.04463531) * distance.pow(3) * vy.pow(3) +
        (-0.25015032) * distance.pow(2) * vx.pow(4) +
        (0.11039557) * distance.pow(2) * vx.pow(3) * vy +
        (-0.03417543) * distance.pow(2) * vx.pow(2) * vy.pow(2) +
        (-0.14394627) * distance.pow(2) * vx * vy.pow(3) +
        (0.00721363) * distance.pow(2) * vy.pow(4) +
        (0.09320355) * distance * vx.pow(5) +
        (-0.02677019) * distance * vx.pow(4) * vy +
        (-0.00108836) * distance * vx.pow(3) * vy.pow(2) +
        (0.08085085) * distance * vx.pow(2) * vy.pow(3) +
        (-0.01474731) * distance * vx * vy.pow(4) +
        (-0.01946372) * distance * vy.pow(5) +
        (-0.00971221) * vx.pow(6) +
        (0.00180160) * vx.pow(5) * vy +
        (0.00331339) * vx.pow(4) * vy.pow(2) +
        (-0.00667505) * vx.pow(3) * vy.pow(3) +
        (0.00385308) * vx.pow(2) * vy.pow(4) +
        (0.01003087) * vx * vy.pow(5) +
        (-0.00041305) * vy.pow(6) +
        (0.00033626) * distance.pow(7) +
        (0.00849543) * distance.pow(6) * vx +
        (-0.00269887) * distance.pow(6) * vy +
        (-0.01165262) * distance.pow(5) * vx.pow(2) +
        (-0.00283887) * distance.pow(5) * vx * vy +
        (-0.00182456) * distance.pow(5) * vy.pow(2) +
        (-0.01338292) * distance.pow(4) * vx.pow(3) +
        (0.01290861) * distance.pow(4) * vx.pow(2) * vy +
        (-0.00283162) * distance.pow(4) * vx * vy.pow(2) +
        (-0.00335357) * distance.pow(4) * vy.pow(3) +
        (0.03028411) * distance.pow(3) * vx.pow(4) +
        (-0.01264162) * distance.pow(3) * vx.pow(3) * vy +
        (0.00437625) * distance.pow(3) * vx.pow(2) * vy.pow(2) +
        (0.01636129) * distance.pow(3) * vx * vy.pow(3) +
        (-0.00074171) * distance.pow(3) * vy.pow(4) +
        (-0.01747134) * distance.pow(2) * vx.pow(5) +
        (0.00498645) * distance.pow(2) * vx.pow(4) * vy +
        (0.00025489) * distance.pow(2) * vx.pow(3) * vy.pow(2) +
        (-0.01379144) * distance.pow(2) * vx.pow(2) * vy.pow(3) +
        (0.00256615) * distance.pow(2) * vx * vy.pow(4) +
        (0.00343650) * distance.pow(2) * vy.pow(5) +
        (0.00363369) * distance * vx.pow(6) +
        (-0.00077215) * distance * vx.pow(5) * vy +
        (-0.00125808) * distance * vx.pow(4) * vy.pow(2) +
        (0.00226028) * distance * vx.pow(3) * vy.pow(3) +
        (-0.00137479) * distance * vx.pow(2) * vy.pow(4) +
        (-0.00350828) * distance * vx * vy.pow(5) +
        (0.00013954) * distance * vy.pow(6) +
        (-0.00020681) * vx.pow(7) +
        (0.00004785) * vx.pow(6) * vy +
        (0.00024129) * vx.pow(5) * vy.pow(2) +
        (0.00029225) * vx.pow(4) * vy.pow(3) +
        (0.00009802) * vx.pow(3) * vy.pow(4) +
        (0.00018793) * vx.pow(2) * vy.pow(5) +
        (-0.00005874) * vx * vy.pow(6) +
        (-0.00007917) * vy.pow(7) +
        (-0.00001749) * distance.pow(8) +
        (-0.00025705) * distance.pow(7) * vx +
        (0.00009341) * distance.pow(7) * vy +
        (0.00050248) * distance.pow(6) * vx.pow(2) +
        (0.00004108) * distance.pow(6) * vx * vy +
        (0.00007261) * distance.pow(6) * vy.pow(2) +
        (0.00034778) * distance.pow(5) * vx.pow(3) +
        (-0.00041332) * distance.pow(5) * vx.pow(2) * vy +
        (0.00010468) * distance.pow(5) * vx * vy.pow(2) +
        (0.00009710) * distance.pow(5) * vy.pow(3) +
        (-0.00136340) * distance.pow(4) * vx.pow(4) +
        (0.00054789) * distance.pow(4) * vx.pow(3) * vy +
        (-0.00021852) * distance.pow(4) * vx.pow(2) * vy.pow(2) +
        (-0.00071557) * distance.pow(4) * vx * vy.pow(3) +
        (0.00002758) * distance.pow(4) * vy.pow(4) +
        (0.00108942) * distance.pow(3) * vx.pow(5) +
        (-0.00030911) * distance.pow(3) * vx.pow(4) * vy +
        (-0.00001255) * distance.pow(3) * vx.pow(3) * vy.pow(2) +
        (0.00081128) * distance.pow(3) * vx.pow(2) * vy.pow(3) +
        (-0.00015134) * distance.pow(3) * vx * vy.pow(4) +
        (-0.00020913) * distance.pow(3) * vy.pow(5) +
        (-0.00033892) * distance.pow(2) * vx.pow(6) +
        (0.00007922) * distance.pow(2) * vx.pow(5) * vy +
        (0.00011864) * distance.pow(2) * vx.pow(4) * vy.pow(2) +
        (-0.00019855) * distance.pow(2) * vx.pow(3) * vy.pow(3) +
        (0.00012576) * distance.pow(2) * vx.pow(2) * vy.pow(4) +
        (0.00032070) * distance.pow(2) * vx * vy.pow(5) +
        (-0.00001151) * distance.pow(2) * vy.pow(6) +
        (0.00003777) * distance * vx.pow(7) +
        (-0.00001086) * distance * vx.pow(6) * vy +
        (-0.00004459) * distance * vx.pow(5) * vy.pow(2) +
        (-0.00004524) * distance * vx.pow(4) * vy.pow(3) +
        (-0.00002061) * distance * vx.pow(3) * vy.pow(4) +
        (-0.00003940) * distance * vx.pow(2) * vy.pow(5) +
        (0.00001161) * distance * vx * vy.pow(6) +
        (0.00001591) * distance * vy.pow(7) +
        (-0.00000344) * vx.pow(8) +
        (-0.00000210) * vx.pow(7) * vy +
        (0.00000317) * vx.pow(6) * vy.pow(2) +
        (0.00000747) * vx.pow(5) * vy.pow(3) +
        (-0.00000596) * vx.pow(4) * vy.pow(4) +
        (-0.00003610) * vx.pow(3) * vy.pow(5) +
        (0.00000027) * vx.pow(2) * vy.pow(6) +
        (0.00000749) * vx * vy.pow(7) +
        (-0.00000120) * vy.pow(8)

fun calculateAngularVelocity(linearVelocity: Double): Double =
    11.086857 * linearVelocity + -50.307666
