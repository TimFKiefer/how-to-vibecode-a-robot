// SPDX-License-Identifier: MIT AND CC-BY-SA-4.0
// Takt parts list (bill of materials).
// Shops and prices for Germany, euros incl. VAT, read on 10 November 2026, not checked again since.
// All money is in euro cents. unit and sum are null where no price was readable: those rows count in no sum.
// check: "page" = the product page was open, "list" = the price stood in the shop's search or category list,
// "estimate" = worked out from the print service's published price rule, no quote, "none" = not checked.
// route: a shop with a route counts only on that route ("printer" = own printer, "service" = print service).

export const READ_ON = "10 November 2026";

export const TIERS = {
  core: { name: "Core", group: "core", what: "the robot walks" },
  face: { name: "Face & voice", group: "face", what: "display, camera, sound" },
  charge: { name: "Everything", group: "all", what: "charging & safety" },
  tools: { name: "Everything", group: "all", what: "tools" },
};

export const CHECKS = {
  page: "product page",
  list: "shop list",
  estimate: "estimate",
  none: "not checked",
};

// Shop totals as the shops' pages print them (goods, shipping, together). null shipping = not checked.
export const SHOPS = [
  { id: "print1", name: "Printed parts · order 1: the fit test", shop: "3DZelle", route: "service",
    note: "Estimate, no quote: worked out from 3DZelle's price rule (PETG from €0.35/g, TPU from €0.45/g; \"from\" means the lowest price) and the weight from the model, without support material. Minimum order €20, shipping \"from €6.99\" (counted as €6.99), free from €99, made in 1 to 7 working days. Order this first: the fit gauge, the fit collar and one whole leg.",
    shipUrl: "https://3dzelle.de/3d-druck-preise/", goods: 2992, ship: 699, total: 3691 },
  { id: "print2", name: "Printed parts · order 2: the full set", shop: "3DZelle", route: "service",
    note: "Order only once the fit test fits. Until then the parts still change. Same sum as above: estimate, no quote.",
    shipUrl: "https://3dzelle.de/3d-druck-preise/", goods: 23203, ship: 0, total: 23203 },
  { id: "jake", name: "3DJake (own printer)", shop: "3DJake", route: "printer",
    note: "Ships from Austria for €5.79, free from €52.90, delivered 3 working days after shipping. Replaces both print orders.",
    shipUrl: "https://www.3djake.de/info/versand-und-lieferung", goods: 21598, ship: 0, total: 21598 },
  { id: "eckstein", name: "Eckstein Komponente", shop: "Eckstein",
    note: "DHL from €6.50, free from €200 of goods. Delivery in 1 to 3 working days.",
    shipUrl: "https://eckstein-shop.de/Versandinformationen", goods: 59892, ship: 0, total: 59892 },
  { id: "reichelt", name: "Reichelt", shop: "Reichelt",
    note: "DHL €5.95 (read in the shop that day). Delivery in 1 to 2 working days, some items 3 to 5.",
    shipUrl: null, goods: 14926, ship: 595, total: 15521 },
  { id: "lindinger", name: "Lindinger (lindinger.at)", shop: "Lindinger",
    note: "To Germany €5.99, free from €99. Two items have no product address of their own: search the shop for the article number.",
    shipUrl: "https://www.lindinger.at/de/Modellbau-Lindinger/Versandkosten/", goods: 5740, ship: 599, total: 6339 },
  { id: "sls", name: "SLS (stefansliposhop.de)", shop: "SLS",
    note: "Shipping €5.90. Company holiday 6 to 15 November; orders ship from 16 November 2026.",
    shipUrl: "https://www.stefansliposhop.de/liefer-und-versandkosten:_:1.html", goods: 1940, ship: 590, total: 2530 },
  { id: "welectron", name: "Welectron", shop: "Welectron",
    note: "DHL/DPD €4.95, free from €49 (DPD).",
    shipUrl: "https://www.welectron.com/Shipping-cost", goods: 2090, ship: 495, total: 2585 },
  { id: "berrybase", name: "Berrybase", shop: "Berrybase",
    note: "Shipping not checked (the shipping page did not open; a search result says €4.95). Only needed for face & voice.",
    shipUrl: null, goods: 2270, ship: null, total: 2270 },
  { id: "none", name: "No shop yet", shop: "",
    note: "No product with a price readable that day. These rows count in no sum.",
    shipUrl: null, goods: 0, ship: 0, total: 0 },
];

// The totals table. estimated = part of the sum that is an estimate; unchecked = not checked, add about.
export const TOTALS = [
  { scope: "Core", what: "the robot walks", route: "printer", goods: 92877, ship: 2279, total: 95156, estimated: 0, unchecked: 0 },
  { scope: "Core", what: "the robot walks", route: "service", goods: 97474, ship: 2978, total: 100452, estimated: 26195, unchecked: 0 },
  { scope: "Core + face & voice", what: "with display, camera, sound", route: "printer", goods: 98432, ship: 2279, total: 100711, estimated: 0, unchecked: 495 },
  { scope: "Core + face & voice", what: "with display, camera, sound", route: "service", goods: 103029, ship: 2978, total: 106007, estimated: 26195, unchecked: 495 },
  { scope: "Everything", what: "plus charging, safety, tools", route: "printer", goods: 108456, ship: 2279, total: 110735, estimated: 0, unchecked: 495 },
  { scope: "Everything", what: "plus charging, safety, tools", route: "service", goods: 113053, ship: 2978, total: 116031, estimated: 26195, unchecked: 495 },
];

// group: what a part does, only a label for the page (added on the website; the totals group by shop).
export const GROUPS = [
  { id: "motion", name: "Motion", what: "servos and the servo bus" },
  { id: "brain", name: "Brain & sensors", what: "computer, storage, orientation sensor, signal wires" },
  { id: "power", name: "Power & wiring", what: "battery, 5 V regulator, plugs, fuse, power wire" },
  { id: "printed", name: "Printed parts", what: "the body and legs, or a printer and filament" },
  { id: "small", name: "Screws & inserts", what: "threaded inserts, screws, nuts" },
  { id: "face", name: "Face & voice", what: "display, camera, speaker, microphone" },
  { id: "care", name: "Charging, safety & tools", what: "charger, fire-safe bag, battery warner, tools" },
];

// id: a short name that stays, used by the build guide (guide-steps.js) to say which step uses a line.
// pack: pieces in one pack, for a line that buys packs (n packs, n x pack pieces).
export const ITEMS = [
  {"id": "p1_gauge", "group": "printed", "shop": "print1", "part": "Fit gauge (fit_gauge)", "purpose": "PETG, 4.7 g each · file fit_gauge_x1.stl", "tier": "core", "n": 1, "unit": 164, "sum": 164, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p1_collar", "group": "printed", "shop": "print1", "part": "Fit collar (fit_collar)", "purpose": "PETG, 3.3 g each · file fit_collar_x1.stl", "tier": "core", "n": 1, "unit": 115, "sum": 115, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p1_coxa", "group": "printed", "shop": "print1", "part": "Hip part (coxa)", "purpose": "PETG, 18.2 g each · file coxa_x6.stl", "tier": "core", "n": 1, "unit": 637, "sum": 637, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p1_femur", "group": "printed", "shop": "print1", "part": "Thigh (femur)", "purpose": "PETG, 18.2 g each · file femur_x6.stl", "tier": "core", "n": 1, "unit": 637, "sum": 637, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p1_knee", "group": "printed", "shop": "print1", "part": "Knee part (knee)", "purpose": "PETG, 17.6 g each · file knee_x6.stl", "tier": "core", "n": 1, "unit": 616, "sum": 616, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p1_blade", "group": "printed", "shop": "print1", "part": "Blade, the lower leg (blade)", "purpose": "PETG, 21.2 g each · file blade_x6.stl", "tier": "core", "n": 1, "unit": 742, "sum": 742, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p1_foot", "group": "printed", "shop": "print1", "part": "Soft foot (foot)", "purpose": "TPU 95A, 1.8 g each · file foot_x6.stl", "tier": "core", "n": 1, "unit": 81, "sum": 81, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p2_base", "group": "printed", "shop": "print2", "part": "Body bottom (base)", "purpose": "PETG, 138.7 g each · file base_x1.stl", "tier": "core", "n": 1, "unit": 4854, "sum": 4854, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p2_lid", "group": "printed", "shop": "print2", "part": "Lid (lid)", "purpose": "PETG, 43.9 g each · file lid_x1.stl", "tier": "core", "n": 1, "unit": 1536, "sum": 1536, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p2_deck", "group": "printed", "shop": "print2", "part": "Electronics deck (deck)", "purpose": "PETG, 7.3 g each · file deck_x1.stl", "tier": "core", "n": 1, "unit": 255, "sum": 255, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p2_hatch", "group": "printed", "shop": "print2", "part": "Belly hatch (hatch)", "purpose": "PETG, 8.0 g each · file hatch_x1.stl", "tier": "core", "n": 1, "unit": 280, "sum": 280, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p2_coxa", "group": "printed", "shop": "print2", "part": "Hip part (coxa)", "purpose": "PETG, 18.2 g each · file coxa_x6.stl", "tier": "core", "n": 6, "unit": 637, "sum": 3822, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p2_femur", "group": "printed", "shop": "print2", "part": "Thigh (femur)", "purpose": "PETG, 18.2 g each · file femur_x6.stl", "tier": "core", "n": 6, "unit": 637, "sum": 3822, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p2_knee", "group": "printed", "shop": "print2", "part": "Knee part (knee)", "purpose": "PETG, 17.6 g each · file knee_x6.stl", "tier": "core", "n": 6, "unit": 616, "sum": 3696, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p2_blade", "group": "printed", "shop": "print2", "part": "Blade, the lower leg (blade)", "purpose": "PETG, 21.2 g each · file blade_x6.stl", "tier": "core", "n": 6, "unit": 742, "sum": 4452, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "p2_foot", "group": "printed", "shop": "print2", "part": "Soft foot (foot)", "purpose": "TPU 95A, 1.8 g each · file foot_x6.stl", "tier": "core", "n": 6, "unit": 81, "sum": 486, "check": "estimate", "url": "https://3dzelle.de/3d-druck-preise/"},
  {"id": "printer", "group": "printed", "shop": "jake", "part": "Bambu Lab A1 mini (3D printer, 180 x 180 x 180 mm)", "purpose": "Prints every part yourself, every repeat too. A placeholder until the printer model is chosen. On 10 Nov 2026: €179.00 instead of €319.00, in stock.", "tier": "core", "n": 1, "unit": 17900, "sum": 17900, "check": "page", "url": "https://www.3djake.de/bambu-lab/a1-mini"},
  {"id": "petg", "group": "printed", "shop": "jake", "part": "The Filament PETG Midnight Black, 1 kg", "purpose": "All hard printed parts (about 730 g with the fit test, plus supports).", "tier": "core", "n": 1, "unit": 1599, "sum": 1599, "check": "page", "url": "https://www.3djake.de/the-filament/petg-midnight-black"},
  {"id": "tpu", "group": "printed", "shop": "jake", "part": "3DJAKE TPU A95 black, 750 g", "purpose": "The soft feet (only about 13 g needed, with the fit test).", "tier": "core", "n": 1, "unit": 2099, "sum": 2099, "check": "page", "url": "https://www.3djake.de/3djake/tpu-a95-schwarz"},
  {"id": "servo", "group": "motion", "shop": "eckstein", "part": "Feetech STS3215 12 V (ST-3215-C018), 30 kg·cm", "purpose": "The servo that drives the joints, 3 per leg.", "tier": "core", "n": 18, "unit": 2645, "sum": 47610, "check": "page", "url": "https://eckstein-shop.de/feetech-st-3215-c018-servo-en"},
  {"id": "servo_spare", "group": "motion", "shop": "eckstein", "part": "Feetech STS3215 12 V (ST-3215-C018), spare", "purpose": "In reserve, in case a servo fails.", "tier": "core", "n": 2, "unit": 2645, "sum": 5290, "check": "page", "url": "https://eckstein-shop.de/feetech-st-3215-c018-servo-en"},
  {"id": "adapter", "group": "motion", "shop": "eckstein", "part": "Waveshare Bus Servo Adapter (A)", "purpose": "Connects the Pi to the servo bus. Hole spacing 37 x 28 mm: needs a CAD change.", "tier": "core", "n": 1, "unit": 595, "sum": 595, "check": "page", "url": "https://eckstein-shop.de/WaveShare-Serial-Bus-Servo-Driver-Board-for-ST-SC-Serial-Bus-Servos-EN"},
  {"id": "imu", "group": "brain", "shop": "eckstein", "part": "Adafruit BNO085 (orientation sensor, I2C)", "purpose": "Measures tilt and turning for the walking control. No room in the model yet: needs a CAD change.", "tier": "core", "n": 1, "unit": 3295, "sum": 3295, "check": "page", "url": "https://eckstein-shop.de/Adafruit-9-DOF-Orientation-IMU-Fusion-Breakout-BNO085-BNO080-EN"},
  {"id": "reg5", "group": "power", "shop": "eckstein", "part": "Pololu D36V28F5 (5 V, 3.2 A)", "purpose": "Makes 5 V for the Pi from the battery. 2.5 mm longer than the space drawn for it.", "tier": "core", "n": 1, "unit": 2612, "sum": 2612, "check": "page", "url": "https://eckstein-shop.de/Pololu-5V-32A-Step-Down-Voltage-Regulator-D36V28F5-Input-Voltage-up-to-50V-EN"},
  {"id": "jumpers", "group": "brain", "shop": "eckstein", "part": "Jumper wires female to female, 40 pieces, 15 cm", "purpose": "Signal lines from the Pi to the adapter, the orientation sensor and the display.", "tier": "core", "n": 1, "pack": 40, "unit": 195, "sum": 195, "check": "page", "url": "https://eckstein-shop.de/AdafruitPremiumFemaleFemaleJumperWires-40x622EN"},
  {"id": "speaker", "group": "face", "shop": "eckstein", "part": "Adafruit mini speaker 8 Ω 0.5 W (1890)", "purpose": "The voice. Check the stock on the product page.", "tier": "face", "n": 1, "unit": 295, "sum": 295, "check": "page", "url": "https://eckstein-shop.de/AdafruitMiniMetalSpeakerwWires-8ohm05WEN"},
  {"id": "ins_m3", "group": "small", "shop": "reichelt", "part": "ruthex threaded inserts M3 x 5.7, 100 pieces", "purpose": "Threads for the blades, the lid and the hatch (20 needed).", "tier": "core", "n": 1, "pack": 100, "unit": 899, "sum": 899, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/3d_druck_gewindeeinsaetze_m3x5_7_100_stueck-332213"},
  {"id": "ins_m25", "group": "small", "shop": "reichelt", "part": "ruthex threaded inserts M2.5 x 5.7, 70 pieces", "purpose": "Threads in the deck posts (4 needed). 3 to 5 working days. The hole size in the model (3.6 mm) is not checked.", "tier": "core", "n": 1, "pack": 70, "unit": 899, "sum": 899, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/3d_druck_gewindeeinsaetze_m2_5x5_7_70_stueck-332212"},
  {"id": "m3x20", "group": "small", "shop": "reichelt", "part": "Cheese-head screws M3 x 20 (slotted), 100 pieces", "purpose": "Lid (4) and feet (6). For the feet the design notes say M3 x 16; M3 x 20 is the safe length.", "tier": "core", "n": 1, "pack": 100, "unit": 680, "sum": 680, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/zylinderschraube_mit_schlitz_m3x20_100er_pack-433667"},
  {"id": "nut_m3", "group": "small", "shop": "reichelt", "part": "Lock nuts M3, 100 pieces", "purpose": "Feet (6 needed).", "tier": "core", "n": 1, "pack": 100, "unit": 345, "sum": 345, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/sicherungsmuttern_m3_100_stueck-65808"},
  {"id": "sd", "group": "brain", "shop": "reichelt", "part": "microSD card SanDisk 32 GB", "purpose": "Storage for the Pi.", "tier": "core", "n": 1, "unit": 1595, "sum": 1595, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/microsdhc-speicherkarte_32gb_sandisk-100189"},
  {"id": "xt60", "group": "power", "shop": "reichelt", "part": "XT60 plug + socket (pair)", "purpose": "Battery connector. Pulling the plug is the main switch (1 + 1 spare).", "tier": "core", "n": 2, "unit": 199, "sum": 398, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/steckverbinder_fuer_li-polymer-akkus_2-polig_xt60-207648"},
  {"id": "fuse", "group": "power", "shop": "reichelt", "part": "Car blade fuse 15 A", "purpose": "Protects the battery and the wires (1 + 2 spare).", "tier": "core", "n": 3, "unit": 36, "sum": 108, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/kfz-flachsicherung_15a_hellblau-7783"},
  {"id": "fuse_holder", "group": "power", "shop": "reichelt", "part": "Car fuse holder, max. 30 A", "purpose": "Holds the fuse.", "tier": "core", "n": 1, "unit": 260, "sum": 260, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/kfz-sicherungshalter_anreihbar_max_30_a-7800"},
  {"id": "molex", "group": "motion", "shop": "reichelt", "part": "Molex Mini-SPOX pin header, 3-pin (22035035)", "purpose": "Sockets on the bus distribution board (7 + 1 spare). Fits the 5264 plug of the servo cables.", "tier": "core", "n": 8, "unit": 24, "sum": 192, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/molex_stiftleiste_-_mini-spox_-_1x3-polig_-_stecker-185732"},
  {"id": "stripboard", "group": "motion", "shop": "reichelt", "part": "Stripboard 50 x 100 mm", "purpose": "Bus distribution for the six legs, soldered by hand.", "tier": "core", "n": 1, "unit": 86, "sum": 86, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/streifenrasterplatine_hartpapier_50x100mm-8275"},
  {"id": "camera", "group": "face", "shop": "reichelt", "part": "Raspberry Pi Camera Module 3 (12 MP, 76°)", "purpose": "The eye. Do not take the €14.60 camera: that is only the sensor without a board.", "tier": "face", "n": 1, "unit": 2990, "sum": 2990, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/raspberry_pi_-_kamera_12mp_76_v3-339256"},
  {"id": "charger", "group": "care", "shop": "reichelt", "part": "Charger M4 Pocket, 1–4S, 5 A, 80 W", "purpose": "Charges the 3S battery with balancing. 3 to 5 working days. Check on the product page which power supply it needs.", "tier": "charge", "n": 1, "unit": 3495, "sum": 3495, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/ladegeraet_fuer_akkupacks_m4_pocket_fuer_1-4s_max_5_a_80_w_lip-364398"},
  {"id": "bag", "group": "care", "shop": "reichelt", "part": "LiPo fire-safe bag LIPO GUARD XL", "purpose": "Charge and store the battery safely. 3 to 5 working days.", "tier": "charge", "n": 1, "unit": 1050, "sum": 1050, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/brandschutzbeutel_fuer_li-polymer-akkus-207644"},
  {"id": "tip_set", "group": "care", "shop": "reichelt", "part": "ruthex soldering tip set for threaded inserts, 8 pieces", "purpose": "Melts the inserts in straight. Before you buy, check that the set fits your soldering iron.", "tier": "tools", "n": 1, "pack": 8, "unit": 1699, "sum": 1699, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/3d_druck_loetspitzen_einschmelzhilfe_set_8_stueck-398415"},
  {"id": "shrink", "group": "care", "shop": "reichelt", "part": "Heat-shrink tubing assortment, 50 pieces", "purpose": "Insulates the solder joints.", "tier": "tools", "n": 1, "pack": 50, "unit": 230, "sum": 230, "check": "list", "url": "https://www.reichelt.de/de/de/shop/produkt/schrumpfschlauch_sortiment_-_50-teilig_farbig_tuete-167392"},
  {"id": "st2x6", "group": "small", "shop": "lindinger", "part": "Robbe self-tapping screws 2.0 x 6 mm, 30 pieces (art. 9796474)", "purpose": "Servo housing S1 (24), camera, adapter.", "tier": "core", "n": 2, "pack": 30, "unit": 189, "sum": 378, "check": "page", "url": "https://lindinger.at/en/SUPPLIES-MISC/BUILDING-ACCESSORIES-SMALL-PARTS/Screws-Nuts-Spacers/Wood-Metal-Screws/Robbe-Modellsport-Tapping-screws-flat-head-Phillips-2.0x6mm-30pcs.-stainless-steel/9796474"},
  {"id": "st2x8", "group": "small", "shop": "lindinger", "part": "Robbe self-tapping screws 2.0 x 8 mm, 30 pieces (art. 9796282)", "purpose": "Servo housings S2 and S3 (48). The link goes to the category: search there for 9796282.", "tier": "core", "n": 2, "pack": 30, "unit": 189, "sum": 378, "check": "list", "url": "https://www.lindinger.at/de/Bauzubehoer-Mehr/Bauzubehoer-Kleinteile/Schrauben-Muttern-Scheiben/Schrauben-Holz-Blech/"},
  {"id": "m3x6", "group": "small", "shop": "lindinger", "part": "Robbe socket-head screws M3 x 6, 30 pieces (art. 9796360)", "purpose": "Reserve for the horn screws. It is not certain that the 144 horn screws come with the servos: check the servo box.", "tier": "core", "n": 1, "pack": 30, "unit": 229, "sum": 229, "check": "page", "url": "https://www.lindinger.at/en/SUPPLIES-MISC/BUILDING-ACCESSORIES-SMALL-PARTS/Screws-Nuts-Spacers/Metric-Screws/Robbe-Modellsport-Hexagon-socket-head-cap-screws-M3x6mm-30pcs.-stainless-steel/9796360"},
  {"id": "m3x6cs", "group": "small", "shop": "lindinger", "part": "Robbe countersunk screws M3 x 6, 30 pieces (art. 9796382)", "purpose": "Belly hatch (4).", "tier": "core", "n": 1, "pack": 30, "unit": 189, "sum": 189, "check": "page", "url": "https://www.lindinger.at/de/Bauzubehoer-Mehr/Bauzubehoer-Kleinteile/Schrauben-Muttern-Scheiben/Schrauben-metrisch/Robbe-Modellsport-Senkkopfschrauben-Innensechskant-M3-0x6mm-30Stk.-Edelstahl/9796382"},
  {"id": "m3x12cs", "group": "small", "shop": "lindinger", "part": "Planet-Hobby countersunk screws M3 x 12, 10 pieces (art. 34744)", "purpose": "Blades (12). Only a few left in stock. The link goes to the category: search there for 34744.", "tier": "core", "n": 2, "pack": 10, "unit": 199, "sum": 398, "check": "list", "url": "https://www.lindinger.at/de/Bauzubehoer-Mehr/Bauzubehoer-Kleinteile/Schrauben-Muttern-Scheiben/Schrauben-metrisch/"},
  {"id": "m25x6", "group": "small", "shop": "lindinger", "part": "Robbe socket-head screws M2.5 x 6, 30 pieces (art. 9796354)", "purpose": "Deck (4) and Pi (4). The link goes to the shop search.", "tier": "core", "n": 1, "pack": 30, "unit": 219, "sum": 219, "check": "list", "url": "https://www.lindinger.at/de/search?search=M2,5x6"},
  {"id": "wire", "group": "power", "shop": "lindinger", "part": "Silicone wire 1.5 mm² red/black, 1 m each (art. 20480)", "purpose": "Power wiring: battery, fuse, distribution board.", "tier": "core", "n": 1, "unit": 399, "sum": 399, "check": "page", "url": "https://www.lindinger.at/de/RC-Elektronik/Kabel-Stecker-Buchsen/Silikonkabel-Hochstromkabel/Planet-Hobby-SILIKONKABEL-1-5MM2-ROT-SCHWARZ-JE-1LFM/20480"},
  {"id": "checker", "group": "care", "shop": "lindinger", "part": "Multiplex Lithium Battery Checker (art. 9771625)", "purpose": "Beeps before the battery runs too low. The only warner whose function was confirmed on that day. Simple beepers are cheaper, but no readable offer was found.", "tier": "charge", "n": 1, "unit": 3550, "sum": 3550, "check": "page", "url": "https://www.lindinger.at/de/RC-Elektronik/Antriebe/Akkus/MULTIPLEX-LITHIUM-BATTERY-CHECKER-MODELLFINDER/9771625"},
  {"id": "battery", "group": "power", "shop": "sls", "part": "SLS XTRON 1000 mAh 3S 11.1 V 50C (XT60)", "purpose": "The power source, 72 x 35 x 18 mm. 3 mm longer and 1 mm thicker than drawn: the battery bay needs a CAD change.", "tier": "core", "n": 1, "unit": 1940, "sum": 1940, "check": "page", "url": "https://www.stefansliposhop.de/akkus/sls-xtron/sls-xtron-50c/sls-xtron-1000mah-3s1p-11-1v-50c-100c::2140.html"},
  {"id": "pi", "group": "brain", "shop": "welectron", "part": "Raspberry Pi Zero 2 W with header (WH)", "purpose": "The computer inside the robot. Status on that day: \"coming soon\"; sold out at four other shops.", "tier": "core", "n": 1, "unit": 2090, "sum": 2090, "check": "page", "url": "https://www.welectron.com/Raspberry-Pi-Zero-2-W-with-Headers_1"},
  {"id": "oled", "group": "face", "shop": "berrybase", "part": "OLED 1.3 inch SH1106 128 x 64, I2C", "purpose": "The face (the eyes). About 36 x 34 mm; drawn as 35.4 x 33.5 mm.", "tier": "face", "n": 1, "unit": 670, "sum": 670, "check": "page", "url": "https://www.berrybase.de/en/1.3-128x64-oled-display-sh1106-iic-i2c-interface-single-colour-blue"},
  {"id": "amp", "group": "face", "shop": "berrybase", "part": "Adafruit MAX98357A amplifier (3006)", "purpose": "Drives the speaker.", "tier": "face", "n": 1, "unit": 590, "sum": 590, "check": "page", "url": "https://www.berrybase.de/en/adafruit-i2s-3w-class-d-amplifier-breakout-max98357a"},
  {"id": "mic", "group": "face", "shop": "berrybase", "part": "Adafruit SPH0645 microphone (3421)", "purpose": "The ear.", "tier": "face", "n": 1, "unit": 890, "sum": 890, "check": "page", "url": "https://www.berrybase.de/en/adafruit-i2s-mems-microphone-breakout-sph0645lm4h"},
  {"id": "cam_cable", "group": "face", "shop": "berrybase", "part": "Camera cable for Pi Zero (standard to mini), 200 mm", "purpose": "Connects the camera to the Pi Zero. Pick the 200 mm length on the product page.", "tier": "face", "n": 1, "unit": 120, "sum": 120, "check": "list", "url": "https://www.berrybase.de/en/raspberry-pi-camera-cable-standard-mini-500mm"},
  {"id": "acrylic", "group": "face", "shop": "none", "part": "Smoke-grey acrylic 34 x 28 x 2 mm", "purpose": "Window in front of the display. A cut piece or an offcut; no ready product found.", "tier": "face", "n": 1, "unit": null, "sum": null, "check": "none", "url": null},
  {"id": "led", "group": "face", "shop": "none", "part": "LED 3 mm with resistor", "purpose": "Status light.", "tier": "face", "n": 1, "unit": null, "sum": null, "check": "none", "url": null},
  {"id": "drivers", "group": "care", "shop": "none", "part": "Small screwdrivers, hex 2.0 and 2.5 mm", "purpose": "For the M2 and M3 screws.", "tier": "tools", "n": 1, "unit": null, "sum": null, "check": "none", "url": null},
  {"id": "iron", "group": "care", "shop": "none", "part": "Soldering iron and solder", "purpose": "Only if you do not have them.", "tier": "tools", "n": 1, "unit": null, "sum": null, "check": "none", "url": null},
];
