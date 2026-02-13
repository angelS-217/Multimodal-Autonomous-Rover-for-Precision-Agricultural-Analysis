#set document(title: "EE 175A WPR Week 4")



#show math.equation: set text(font: "New Computer Modern Math")
#show math.equation: it => {
  if it.has("label") { 
    math.equation(
      block:true,
      numbering: "(1)", 
      it
    ) 
  } else { it
    } 
}
#show outline.entry.where(
  level: 1
): it => {
    v(12pt, weak: false)
    strong(it)
  }
#show raw.where(block: true): it => {
  align(center,
    block(
      fill: luma(240),
      stroke: 1pt + black,
      inset: 10pt,
      radius: 4pt,
      it  // 'it' is the original code block
    )
  )
}
#set page(
  numbering: "1",
  number-align: center,
  margin: (x: 0.5in, y: 0.5in),
)
#set text(
  font: "Fira Sans",
  size: 12pt
)
#set heading(numbering: none)
#set math.mat(delim: "[")


/*COVER PAGE*/ 
#align(horizon + center)[
  #text(18pt, weight: "bold")[Autonomous UAV \ for Plant Health Analysis]
  #v(0.1em)
  #text(16pt, weight: "bold")[EE 175A, F25]
  #v(0.1em)
  #text(12pt)[#super[1]Ying Gao, #super[2]Adam Lachman, #super[3]Angel Pulido, ]
  #v(0.1em)
  #text(12pt)[Week 4, 10/20 - 10/26]
]

#align(bottom+center)[
  #text(16pt, weight: "bold")[Executive Summary]
  #v(0.1em)
  
  /*Summary*/
  The goal of this senior design project is to design an autonomous unmanned aerial vehicle quadcopter to monitor plant and crop health. The health analysis of flora will be implemented using the open-source PlantCV Python package. This analysis will be conducted via a camera sensor rigged onto the UAV to capture images of flora. Captured images will be processed using the ML algorithm by an onboard computer. Data packets containing results of post-processed images are then transmitted via onboard LoRa wireless antenna to a ground station system monitor (PC) through a LoRa wireless antenna receiver. \

  Implementation of autonomous navigation will be conducted using a Ground Control Station (GCS) application to define a mission flight path to be uploaded to the drone's flight controller. The autopilot flight controller, executed with PX4 Autopilot software,  will then follow predesignated waypoints and stop at specific points to capture images of flora. \
  
  Design approaches of the UAV quadcopter are to be determined. Approaches we have considered range from purchasing a pre-built drone, constructing custom-built drone, or purchasing an open-source drone kit.
]


// #pagebreak()


// Set justified alignment for all content after title page
#set par(justify: true,
         first-line-indent: 2em)



 #pagebreak()


/*TECHNICAL DISCUSSION*/

// ```matlab
// % rotate line image by 45 degrees
// ln45 = imrotate(lineI, 45, 'nearest', 'crop');
// ln90 = rot90(lineI);  % rotate line image by 90 degrees




// #block(
//   inset: (left: 3.5em),
// )[
//   + 
// ]

// This text is back at the normal margin.



/*IMAGES*/
= Appendix
// 1-2 images per page
#figure(
  image("ee175_sbd_v0-01.png",
        width: 100%),
  caption: [Preliminary System Block Diagram. Component specifications included for team reference and will be removed for the final SBD.]
) <sbd>

