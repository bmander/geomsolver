//! **A STEP file's text read into its entities** (`brep::step_file`; #993 stage 1): the syntax of
//! ISO 10303-21 as writers write it, and the units and uncertainty a file states
//! (`brep::step_read::context`).
//!
//! The syntax: every form a writer may use read as the grammar means it, the data section's
//! answer the same however many runs it is cut into, every error saying where, and no text — a
//! real file truncated or edited at random — making the reader panic. The file: its header and
//! schema, its units followed down to SI by their conversion factors, its uncertainty, which
//! solids it holds and their products' names; and each refusal with its `Why` and its `#n`.
//! The fixtures in `step_file/` are Open CASCADE's (`step_file/make.py`); the rest are written
//! here.

use gcs_core::brep::step_file::{decode,Error,File,Header,Value};
use gcs_core::brep::step_read::context::{self,Quantity,Schema};
use gcs_core::brep::step_read::{Refusal,Why};
use gcs_core::rng::Rng;

const DRILLED_AP203: &str = include_str!("step_file/drilled_ap203.step");
const DRILLED_AP214: &str = include_str!("step_file/drilled_ap214.step");
const DRILLED_AP242: &str = include_str!("step_file/drilled_ap242.step");
const DRILLED_INCH: &str = include_str!("step_file/drilled_inch.step");
const DRILLED_METRE: &str = include_str!("step_file/drilled_metre.step");
const BOX_FOOT: &str = include_str!("step_file/box_ap242_foot.step");
const NAMED: &str = include_str!("step_file/named_ap214.step");
const VOID: &str = include_str!("step_file/void_ap214.step");
const SHELL: &str = include_str!("step_file/shell_ap214.step");
const ASSEMBLY: &str = include_str!("step_file/assembly_ap214.step");
const PLACED: &str = include_str!("step_file/placed_ap214.step");

fn parse(text: &str) -> File<'_> { File::read(text,7,&|_| {}).unwrap_or_else(|e| panic!("{e}")) }
fn fails(text: &str) -> Error { File::read(text,7,&|_| {}).err().expect("the text should not parse") }
fn near(a: f64,b: f64) -> bool { (a-b).abs() <= 1e-12*a.abs().max(b.abs()).max(1.) }

/// A file around `data`, its schema `schema`.
fn file_of(schema: &str,data: &str) -> String {
    format!("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('','',(''),(''),'','','');\n\
        FILE_SCHEMA(('{schema}'));\nENDSEC;\nDATA;\n{data}ENDSEC;\nEND-ISO-10303-21;\n")
}

/// One solid in an advanced B-rep representation, its product named `name one`, its context in
/// millimetres and radians with an uncertainty of a thousandth: the lines of its data section.
const PART: [&str;20] = [
    "#1=PRODUCT('p1','name one','',(#2));",
    "#2=PRODUCT_CONTEXT('',#3,'mechanical');",
    "#3=APPLICATION_CONTEXT('x');",
    "#4=PRODUCT_DEFINITION_FORMATION('','',#1);",
    "#5=PRODUCT_DEFINITION('design','',#4,#6);",
    "#6=PRODUCT_DEFINITION_CONTEXT('part definition',#3,'design');",
    "#7=PRODUCT_DEFINITION_SHAPE('','',#5);",
    "#8=SHAPE_DEFINITION_REPRESENTATION(#7,#10);",
    "#10=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#11,#15),#20);",
    "#11=AXIS2_PLACEMENT_3D('',#12,#13,#14);",
    "#12=CARTESIAN_POINT('',(0.,0.,0.));",
    "#13=DIRECTION('',(0.,0.,1.));",
    "#14=DIRECTION('',(1.,0.,0.));",
    "#15=MANIFOLD_SOLID_BREP('',#16);",
    "#16=CLOSED_SHELL('',());",
    "#20=(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#24)) GLOBAL_UNIT_ASSIGNED_CONTEXT((#21,#22,#23)) REPRESENTATION_CONTEXT('',''));",
    "#21=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));",
    "#22=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));",
    "#23=(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT());",
    "#24=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-03),#21,'distance_accuracy_value','');",
];

/// `PART` with lines replaced (by their instance's `#n=`; an empty replacement drops the line) and
/// lines added, under AP214.
fn part(replace: &[(&str,&str)],add: &[&str]) -> String {
    let mut lines: Vec<String> = PART.iter().filter_map(|l| {
        match replace.iter().find(|(n,_)| l.starts_with(&format!("{n}="))) {
            Some((_,"")) => None,Some((_,r)) => Some(r.to_string()),None => Some(l.to_string()),
        }
    }).collect();
    lines.extend(add.iter().map(|s| s.to_string()));
    file_of("AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }",&(lines.join("\n")+"\n"))
}

fn solids(text: &str) -> Result<Vec<context::Found>,Refusal> { context::solids(&parse(text)) }
fn refused(text: &str) -> Refusal { solids(text).err().expect("the file should be refused") }

/// What a writer may write, read as the grammar means it: a byte-order mark and comments before
/// the file, `DATA;` mid-line, entities several to a line and one over many, comments between
/// any tokens, strings holding a `;`, a doubled quote, `ENDSEC`, a `#`, a `/*` and a line break,
/// reals in each form, `$`, `*`, enumerations, typed parameters, a user-defined keyword, a binary
/// and a complex entity.
#[test]
fn syntax_as_writers_write_it() {
    let text = "\u{feff}  /* a comment before the file */\n ISO-10303-21 ;\nHEADER;FILE_DESCRIPTION(('d'),'2;1');\
        FILE_NAME('n','t',('a'),('o'),'pre','sys','auth');FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));ENDSEC; DATA;#1=A('x;y',\
        'it''s','ENDSEC;','#2=B();','/* no comment */','two\nlines');#2 = B ( 1. , 1.E5 , -.5E-3 , -12 , $ , * , .T. ) ;\n\
        #3\n=\n/* here */ C\n(\n#1\n,\n( #2 , #1 ) ,\nLENGTH_MEASURE ( 1.E-07 ) , \"0FF\" )\n;\
        #4=!USER_KEYWORD(.ENUM_1.,!TYPED(2));#5=(LENGTH_UNIT()NAMED_UNIT(*)/* c */SI_UNIT(.MILLI.,.METRE.));\n\
        ENDSEC;\nEND-ISO-10303-21;\n";
    let f = parse(text);
    assert_eq!(f.entities.len(),5);
    let (a,v) = f.get(1).simple().unwrap();
    assert_eq!(a,"A");
    let strings: Vec<&str> = v.iter().map(|v| v.text().unwrap()).collect();
    assert_eq!(strings,["x;y","it''s","ENDSEC;","#2=B();","/* no comment */","two\nlines"]);
    assert_eq!(v[1].name().unwrap(),"it's");
    assert_eq!(v[5].name().unwrap(),"twolines","a line break is no part of a string");
    let (_,v) = f.get(2).simple().unwrap();
    assert_eq!(v,[Value::Real(1.),Value::Real(1e5),Value::Real(-0.5e-3),Value::Int(-12),Value::Unset,Value::Derived,Value::Enum("T")]);
    assert_eq!(v[6].logical(),Some(true));
    let (c,v) = f.get(3).simple().unwrap();
    assert_eq!(c,"C");
    assert_eq!(v[0],Value::Ref(1));
    assert_eq!(v[1],Value::List(vec![Value::Ref(2),Value::Ref(1)]));
    assert_eq!(v[2],Value::Typed("LENGTH_MEASURE",vec![Value::Real(1e-7)]));
    assert_eq!(v[2].measure(),Some(1e-7));
    assert_eq!(v[3],Value::Str("0FF"));
    let (k,v) = f.get(4).simple().unwrap();
    assert_eq!((k,v),("!USER_KEYWORD",&[Value::Enum("ENUM_1"),Value::Typed("!TYPED",vec![Value::Int(2)])][..]));
    let five = f.get(5);
    assert_eq!(five.kinds(),"LENGTH_UNIT+NAMED_UNIT+SI_UNIT");
    assert_eq!(five.part("SI_UNIT").unwrap(),[Value::Enum("MILLI"),Value::Enum("METRE")]);
    assert_eq!(f.header.schema_names(),["AUTOMOTIVE_DESIGN"]);
}

/// A file's lines may end `\n`, `\r\n` or `\r`: the entities are the same, and an error's line is
/// counted alike.
#[test]
fn line_ends_of_every_kind() {
    let unix = part(&[],&[]);
    let dos = unix.replace('\n',"\r\n");
    let mac = unix.replace('\n',"\r");
    let (a,b,c) = (parse(&unix),parse(&dos),parse(&mac));
    assert_eq!(a.entities,b.entities);
    assert_eq!(a.entities,c.entities);
    let bad = unix.replace("#13=DIRECTION('',(0.,0.,1.));","#13=DIRECTION('',(0.,0.,1.);");
    let lines: Vec<Option<usize>> = [bad.clone(),bad.replace('\n',"\r\n"),bad.replace('\n',"\r")].iter().map(|t| fails(t).line).collect();
    assert_eq!(lines,[Some(19),Some(19),Some(19)],"the error is on the line #13 is written on");
}

/// Reals as the grammar writes them (`1.`, `1.E5`, `-0.5E-3`, a sign), and the forms that have
/// one meaning though the grammar does not write them (`-.5E-3`, `1E5`, `1.e5`); a number the
/// grammar cannot mean is an error naming its entity.
#[test]
fn reals_as_the_grammar_writes_them() {
    let read = |n: &str| -> Result<Value<'static>,Error> {
        let text: &'static str = Box::leak(file_of("AUTOMOTIVE_DESIGN",&format!("#1=X({n});\n")).into_boxed_str());
        File::read(text,1,&|_| {}).map(|f| f.get(1).simple().unwrap().1[0].clone())
    };
    for (n,x) in [("1.",1.),("1.E5",1e5),("-0.5E-3",-0.5e-3),("+2.5",2.5),("0.",0.),("1.5E+02",150.),("-.5E-3",-0.5e-3),("1E5",1e5),
        ("1.e5",1e5),(".25",0.25)] {
        assert_eq!(read(n).unwrap(),Value::Real(x),"{n}");
    }
    assert_eq!(read("-7").unwrap(),Value::Int(-7));
    assert_eq!(read("+7").unwrap(),Value::Int(7));
    for n in ["1.E","-","+.","1.2.3","1E+","99999999999999999999"] {
        let e = read(n).err().unwrap_or_else(|| panic!("{n} should not parse"));
        assert_eq!(e.entity,Some(1),"{n}: {e}");
        assert!(!e.unsupported);
    }
}

/// A string used as a name is decoded: `''`, `\\`, `\S\`, `\X\`, `\X2\…\X0\`, `\X4\…\X0\`, a
/// line break dropped; the raw text stays as written.
#[test]
fn strings_decode_as_names() {
    for (raw,name) in [("it''s","it's"),("a\\\\b","a\\b"),("\\S\\e","\u{e5}"),("caf\\X\\E9","caf\u{e9}"),
        ("\\X2\\00E9006C00E8\\X0\\ve","\u{e9}l\u{e8}ve"),("\\X2\\D83DDE00\\X0\\","\u{1f600}"),("\\X4\\0001F600\\X0\\!","\u{1f600}!"),
        ("one\r\ntwo","onetwo"),("\\N\\","\n"),("plain \\ slash","plain \\ slash"),("\\X2\\00E","\u{fffd}"),("\\PB\\\\S\\e","\u{fffd}"),
        ("\u{e9}t\u{e9}","\u{e9}t\u{e9}")] {
        assert_eq!(decode(raw),name,"{raw:?}");
    }
    let text = file_of("AUTOMOTIVE_DESIGN","#1=X('\\X2\\00C5\\X0\\ngstr\\S\\vm');\n");
    let f = parse(&text);
    assert_eq!(f.get(1).simple().unwrap().1[0],Value::Str("\\X2\\00C5\\X0\\ngstr\\S\\vm"));
    assert_eq!(f.get(1).simple().unwrap().1[0].name().unwrap(),"\u{c5}ngstr\u{f6}m");
}

/// The header: `FILE_DESCRIPTION`, `FILE_NAME` and `FILE_SCHEMA` read and decoded, every header
/// entity kept, the schemas' names without their object identifiers.
#[test]
fn header_and_schema() {
    let f = parse(DRILLED_AP214);
    let h: &Header = &f.header;
    assert_eq!(h.description,["Open CASCADE Model"]);
    assert_eq!(h.level,"2;1");
    assert_eq!(h.name,"Open CASCADE Shape Model");
    assert_eq!(h.preprocessor,"Open CASCADE STEP processor 7.9");
    assert_eq!(h.originating_system,"Open CASCADE 7.9");
    assert_eq!(h.schemas,["AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }"]);
    assert_eq!(h.records.iter().map(|r| r.0).collect::<Vec<_>>(),["FILE_DESCRIPTION","FILE_NAME","FILE_SCHEMA"]);
    for (text,schema,name) in [(DRILLED_AP203,Schema::Ap203,"CONFIG_CONTROL_DESIGN"),(DRILLED_AP214,Schema::Ap214,"AUTOMOTIVE_DESIGN"),
        (DRILLED_AP242,Schema::Ap242,"AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF")] {
        let f = parse(text);
        assert_eq!(f.header.schema_names(),[name]);
        assert_eq!(context::schema(&f),Ok(schema));
    }
    let long = "AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF { 1 0 10303 403 3 1 4 }";
    assert_eq!(context::schema(&parse(&file_of(long,""))),Ok(Schema::Ap203));
    // a schema the reader was not written against is refused, naming it, and so are its solids
    let ifc = part(&[],&[]).replace("AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }","IFC2X3");
    let r = refused(&ifc);
    assert_eq!((r.why,r.entity),(Why::Unsupported,None));
    assert!(r.msg.contains("IFC2X3"),"{r}");
    let none = part(&[],&[]).replace("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));","");
    assert_eq!(refused(&none).why,Why::Malformed);
}

/// Every error says where: the instance it is in where there is one, and its line; what the
/// reader does not read is said so (`unsupported`).
#[test]
fn errors_say_where() {
    let base = part(&[],&[]);
    let e = fails(&base.replace("#13=DIRECTION('',(0.,0.,1.));","#13=DIRECTION('',(0.,0.,1.)"));
    assert_eq!((e.entity,e.unsupported),(Some(13),false),"{e}");
    let e = fails(&base.replace("#14=DIRECTION('',(1.,0.,0.));","#14=DIRECTION('oops,(1.,0.,0.));"));
    assert_eq!(e.entity,Some(14),"{e}");
    assert!(e.to_string().starts_with("#14, line "),"{e}");
    let e = fails(&base.replace("#16=CLOSED_SHELL('',());","#16=CLOSED_SHELL('',(#99));"));
    assert_eq!(e.entity,Some(16),"{e}");
    assert!(e.msg.contains("#99"),"{e}");
    let e = fails(&base.replace("#16=CLOSED_SHELL('',());","#16=CLOSED_SHELL('',());\n#15=X();"));
    assert_eq!(e.entity,Some(15),"{e}");
    let e = fails(&base.replace("#16=CLOSED_SHELL","junk #16=CLOSED_SHELL"));
    assert_eq!((e.entity,e.line),(None,Some(22)),"{e}");
    assert!(e.msg.contains("after #15"),"{e}");
    assert!(fails(&base.replace("END-ISO-10303-21;","")).msg.contains("END-ISO-10303-21"));
    assert!(fails(&format!("{base}#1=X();")).msg.contains("after"));
    assert!(fails(&base.replace("ISO-10303-21;\n","")).msg.contains("ISO-10303-21"));
    assert!(fails(&base.replace("ENDSEC;\nEND","END")).msg.contains("ENDSEC"));
    assert!(fails(&base.replace("/*","").replace("#3=APPLICATION_CONTEXT('x');","#3=APPLICATION_CONTEXT('x'); /* open")).msg.contains("unclosed comment"));
    // what the reader does not read: a second data section, edition 3's sections, a scope, a value instance
    for (text,entity,what) in [
        (base.replace("ENDSEC;\nEND-ISO","ENDSEC;\nDATA;\n#30=X();\nENDSEC;\nEND-ISO"),None,"second DATA"),
        (base.replace("ENDSEC;\nDATA;","ENDSEC;\nANCHOR;\n<a>=#1;\nENDSEC;\nDATA;"),None,"ANCHOR"),
        (base.replace("ENDSEC;\nDATA;","ENDSEC;\nREFERENCE;\n#100=<x.stp#1>;\nENDSEC;\nDATA;"),None,"REFERENCE"),
        (base.replace("ENDSEC;\nEND-ISO","ENDSEC;\nSIGNATURE;\nabc\nENDSEC;\nEND-ISO"),None,"SIGNATURE"),
        (base.replace("#16=CLOSED_SHELL('',());","#16=&SCOPE\n#17=X();\nENDSCOPE CLOSED_SHELL('',());"),Some(16),"SCOPE"),
        (base.replace("#16=CLOSED_SHELL('',());","#16=CLOSED_SHELL('',(@3));"),Some(16),"value instance")] {
        let e = fails(&text);
        assert_eq!((e.unsupported,e.entity),(true,entity),"{what}: {e}");
        assert!(e.msg.contains(what),"{what}: {e}");
        assert!(e.line.is_some(),"{what}: {e}");
        let r = context::parse(&text).err().unwrap();
        assert_eq!((r.why,r.entity),(Why::Unsupported,entity),"{what}: {r}");
    }
    let r = context::parse(&base.replace("#13=DIRECTION('',(0.,0.,1.));","#13=DIRECTION('',(0.,0.,1.)")).err().unwrap();
    assert_eq!((r.why,r.entity),(Why::Malformed,Some(13)),"{r}");
}

/// Instance numbers far apart are as good as close ones.
#[test]
fn instance_numbers_may_be_sparse() {
    let text = file_of("AUTOMOTIVE_DESIGN","#4000000000=X(#7);\n#7=Y(#4000000000);\n#123456=Z();\n");
    let f = parse(&text);
    assert_eq!(f.get(4000000000).simple().unwrap().0,"X");
    assert_eq!(f.get(7).simple().unwrap().1[0],Value::Ref(4000000000));
    assert!(f.find(8).is_none());
    assert_eq!(fails(&text.replace("#123456=","#7=")).entity,Some(7),"defined twice");
    assert!(fails(&file_of("AUTOMOTIVE_DESIGN","#4294967296=X();\n")).msg.contains("instance number"));
}

/// Entities whose strings and comments hold `;`, line breaks and `#`, so that a cut where a `;`
/// is followed by a `#` may fall inside one.
fn treacherous(n: usize) -> String {
    let mut data = String::new();
    for k in 1..=n {
        match k%5 {
            0 => data += &format!("#{k}=S('a;\n#{}=S(''x'');\n#',#{});\n",k+1,k.max(2)-1),
            1 => data += &format!("#{k}=T(1.,'ENDSEC;\n#9=');/* ;\n#{k}=X(); */\n"),
            2 => data += &format!("#{k}=U((#{},#{}),.T.);#{}=V('');\n",k.max(2)-1,k,k+n),
            3 => data += &format!("#{k}=W(\n'x'';\n#'\n);\n"),
            _ => data += &format!("#{k}=X(/*;\n#1=*/$);\n"),
        }
    }
    file_of("AUTOMOTIVE_DESIGN",&data)
}

/// The data section's answer does not depend on how many runs it is cut into: the same entities
/// for a real file, for a text whose strings and comments hold every cut's pattern, and the same
/// error for a text that does not parse.
#[test]
fn answer_independent_of_runs() {
    for text in [DRILLED_AP214.to_string(),VOID.to_string(),treacherous(400)] {
        let one = File::read(&text,1,&|_| {}).unwrap();
        for runs in [2,3,4,7,16,64,1000] {
            let many = File::read(&text,runs,&|_| {}).unwrap();
            assert_eq!(one.entities,many.entities,"{runs} runs");
            assert_eq!(one.header,many.header);
        }
    }
    let t = treacherous(400);
    let f = parse(&t);
    assert_eq!(f.get(5).simple().unwrap().1[0].text(),Some("a;\n#6=S(''x'');\n#"));
    assert_eq!(f.entities.len(),400+80);
    for bad in [t.replace("#200=","#200 "),t.replace("#313=W(\n'x''","#313=W(\n'x'"),t.replace("#399=","#398=")] {
        let one = File::read(&bad,1,&|_| {}).err().unwrap();
        for runs in [2,4,16,64,1000] { assert_eq!(File::read(&bad,runs,&|_| {}).err().unwrap(),one,"{runs} runs"); }
    }
}

/// No text makes the reader panic: real files truncated and edited at random (seeded), each
/// parsed in one run and in several to the same answer, and the ones that parse read for their
/// solids.
#[test]
fn no_panic_under_fuzz() {
    let mut rng = Rng::new(993);
    const SOME: [&str;24] = [";","#","'","''","(",")","/*","*/","\\","\\X2\\","\\X0\\","\r","\n","$","*",".","E","-","=","ENDSEC;","DATA;",
        "\u{e9}","9999999999","#0"];
    let sources = [DRILLED_INCH,VOID,ASSEMBLY,SHELL,&treacherous(60)];
    let mut parsed = 0;
    for k in 0..2000 {
        let src = sources[k%sources.len()];
        let mut text = src.to_string();
        for _ in 0..1+rng.int(3) {
            let mut at = rng.int(text.len()+1);
            while !text.is_char_boundary(at) { at -= 1; }
            match rng.int(4) {
                0 => text.truncate(at),
                1 => { let mut to = (at+1+rng.int(40)).min(text.len()); while !text.is_char_boundary(to) { to += 1; } text.replace_range(at..to,""); }
                2 => text.insert_str(at,SOME[rng.int(SOME.len())]),
                _ => { let c = (b' '+rng.int(95) as u8) as char; let mut to = (at+1).min(text.len()); while !text.is_char_boundary(to) { to += 1; }
                    text.replace_range(at..to,&c.to_string()); }
            }
        }
        let one = File::read(&text,1,&|_| {});
        let many = File::read(&text,5,&|_| {});
        match (&one,&many) {
            (Ok(a),Ok(b)) => { assert_eq!(a.entities,b.entities); parsed += 1; let _ = context::solids(a); }
            (Err(a),Err(b)) => { assert_eq!(a,b); assert!(a.line.is_some() || a.entity.is_some(),"{a}"); }
            _ => panic!("one run and five disagree on edit {k}"),
        }
    }
    assert!(parsed > 20,"{parsed} of the edited texts parsed: the edits should leave some whole");
}

/// The units a context assigns, followed down to SI by their conversion factors and never read
/// by their names: the inch, the foot, the metre, a degree, a unit converted twice, any SI
/// prefix, the measure written complex; the uncertainty in the unit its measure names, and none
/// where none is stated.
#[test]
fn units_by_conversion_factor() {
    let one = |text: &str| -> context::Context { let s = solids(text).unwrap_or_else(|r| panic!("{r}")); assert_eq!(s.len(),1); s[0].context };
    let c = one(DRILLED_INCH);
    assert_eq!((c.length,c.angle),(25.4,1.));
    assert!((c.uncertainty.unwrap()-1e-7).abs() < 1e-18,"{c:?}");
    let c = one(BOX_FOOT);
    assert!(near(c.length,304.8),"{c:?}");
    assert!((c.uncertainty.unwrap()-1e-7).abs() < 1e-18,"{c:?}");
    let c = one(DRILLED_METRE);
    assert_eq!(c.length,1000.);
    assert!((c.uncertainty.unwrap()-1e-7).abs() < 1e-18,"{c:?}");
    let c = one(DRILLED_AP242);
    assert_eq!((c.length,c.angle,c.entity),(1.,1.,429));
    let c = one(&part(&[],&[]));
    assert_eq!((c.length,c.angle,c.uncertainty,c.entity),(1.,1.,Some(1e-3),20));
    // a degree, by its factor; its name says nothing
    let degree = [("#22","#22=(CONVERSION_BASED_UNIT('RADIAN',#25) NAMED_UNIT(#26) PLANE_ANGLE_UNIT());")];
    let c = one(&part(&degree,&["#25=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.0174532925199433),#27);",
        "#26=DIMENSIONAL_EXPONENTS(0.,0.,0.,0.,0.,0.,0.);","#27=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));"]));
    assert!(near(c.angle,std::f64::consts::PI/180.),"{c:?}");
    // a thou: a thousandth of an inch, the inch 2.54 centimetres, the measure written complex
    let thou = [("#21","#21=(CONVERSION_BASED_UNIT('METRE',#25) LENGTH_UNIT() NAMED_UNIT(#26));")];
    let c = one(&part(&thou,&["#25=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.001),#27);","#26=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);",
        "#27=(CONVERSION_BASED_UNIT('INCH',#28) LENGTH_UNIT() NAMED_UNIT(#26));",
        "#28=(LENGTH_MEASURE_WITH_UNIT() MEASURE_WITH_UNIT(LENGTH_MEASURE(2.54),#29));",
        "#29=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.CENTI.,.METRE.));"]));
    assert!(near(c.length,0.0254),"{c:?}");
    assert!(near(c.uncertainty.unwrap(),0.0254e-3),"the uncertainty in thou: {c:?}");
    for (p,mm) in [("$",1e3),(".KILO.",1e6),(".CENTI.",10.),(".DECI.",100.),(".MICRO.",1e-3),(".NANO.",1e-6)] {
        let si = format!("#21=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT({p},.METRE.));");
        assert!(near(one(&part(&[("#21",&si)],&[])).length,mm),"{p}");
    }
    // the uncertainty in a unit of its own, and none stated
    let c = one(&part(&[("#24","#24=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(2.E-06),#25,'distance_accuracy_value','');")],
        &["#25=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.));"]));
    assert!(near(c.uncertainty.unwrap(),2e-3),"{c:?}");
    let c = one(&part(&[("#20","#20=(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#21,#22,#23)) REPRESENTATION_CONTEXT('',''));"),
        ("#24","")],&[]));
    assert_eq!(c.uncertainty,None);
    // a solid angle's unit is not read, whatever it is
    let c = one(&part(&[("#23","#23=(CONTEXT_DEPENDENT_UNIT('sr') NAMED_UNIT(*) SOLID_ANGLE_UNIT());")],&[]));
    assert_eq!(c.length,1.);
    let f = parse(DRILLED_INCH);
    assert_eq!(context::unit(&f,430),Ok(context::Unit {quantity:Quantity::Length,si:0.0254}));
}

/// A unit the reader cannot resolve is refused, naming it: a context-dependent unit, an unknown
/// prefix, a conversion that does not end, a length converted to an angle; a context without a
/// unit of length is malformed.
#[test]
fn units_unresolved_refused() {
    for (replace,add,why,entity) in [
        (("#21","#21=(CONTEXT_DEPENDENT_UNIT('thing') LENGTH_UNIT() NAMED_UNIT(*));"),vec![],Why::Unsupported,21),
        (("#21","#21=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.YOCTO.,.METRE.));"),vec![],Why::Unsupported,21),
        (("#21","#21=(CONVERSION_BASED_UNIT('LOOP',#25) LENGTH_UNIT() NAMED_UNIT(*));"),vec!["#25=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(2.),#21);"],
            Why::Malformed,21),
        (("#21","#21=(CONVERSION_BASED_UNIT('INCH',#25) LENGTH_UNIT() NAMED_UNIT(*));"),vec!["#25=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#22);"],
            Why::Malformed,25),
        (("#21","#21=(CONVERSION_BASED_UNIT('INCH',#25) LENGTH_UNIT() NAMED_UNIT(*));"),vec!["#25=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(-25.4),#26);",
            "#26=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));"],Why::Malformed,25),
        (("#21","#21=(DERIVED_UNIT((#25)) LENGTH_UNIT());"),vec!["#25=X();"],Why::Unsupported,21),
        (("#20","#20=(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#22,#23)) REPRESENTATION_CONTEXT('',''));"),vec![],Why::Malformed,20),
        (("#20","#20=(GEOMETRIC_REPRESENTATION_CONTEXT(2) GLOBAL_UNIT_ASSIGNED_CONTEXT((#21,#22,#23)) REPRESENTATION_CONTEXT('',''));"),vec![],Why::Malformed,20),
    ] {
        let r = refused(&part(&[replace],&add));
        assert_eq!((r.why,r.entity),(why,Some(entity)),"{}: {r}",replace.1);
    }
}

/// The solids a file holds, each once, in the file's order, with its product's name: as Open
/// CASCADE writes them under each schema, with a void, as XCAF names a part, written complex,
/// through a shape representation tied to the solid's without a transformation, and two solids
/// in two contexts each read once.
#[test]
fn solids_found_with_names() {
    for (text,name) in [(DRILLED_AP203,"drilled_ap203 1"),(DRILLED_AP214,"drilled_ap214 2"),(DRILLED_AP242,"drilled_ap242 3"),
        (DRILLED_INCH,"drilled_inch 4"),(NAMED,"bracket"),(VOID,"void_ap214 7")] {
        let s = solids(text).unwrap_or_else(|r| panic!("{name}: {r}"));
        assert_eq!(s.len(),1,"{name}");
        assert_eq!((s[0].solid,&s[0].name[..]),(15,name));
    }
    assert!(parse(VOID).get(15).is("BREP_WITH_VOIDS"));
    let s = solids(&part(&[],&[])).unwrap();
    assert_eq!((s[0].solid,&s[0].name[..]),(15,"name one"));
    // the representation complex, the name escaped
    let s = solids(&part(&[("#10","#10=(ADVANCED_BREP_SHAPE_REPRESENTATION() REPRESENTATION('',(#11,#15),#20) SHAPE_REPRESENTATION());"),
        ("#1","#1=PRODUCT('p1','\\X2\\00C5\\X0\\ngstr\\S\\vm','',(#2));")],&[])).unwrap();
    assert_eq!((s[0].solid,&s[0].name[..]),(15,"\u{c5}ngstr\u{f6}m"));
    // the product's shape representation tied to the B-rep's without a transformation
    let s = solids(&part(&[("#8","#8=SHAPE_DEFINITION_REPRESENTATION(#7,#30);")],&["#30=SHAPE_REPRESENTATION('',(#11),#20);",
        "#31=SHAPE_REPRESENTATION_RELATIONSHIP('','',#30,#10);"])).unwrap();
    assert_eq!(&s[0].name,"name one");
    // a solid in a plain shape representation; none named where no product says
    let s = solids(&part(&[("#10","#10=SHAPE_REPRESENTATION('',(#11,#15),#20);"),("#8","")],&[])).unwrap();
    assert_eq!((s[0].solid,&s[0].name[..]),(15,""));
    // two solids in two representations, two contexts
    let s = solids(&part(&[],&["#40=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#41),#42);","#41=MANIFOLD_SOLID_BREP('',#16);",
        "#42=(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#43,#22)) REPRESENTATION_CONTEXT('',''));",
        "#43=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.CENTI.,.METRE.));"])).unwrap();
    assert_eq!(s.iter().map(|f| (f.solid,f.context.entity,f.context.length)).collect::<Vec<_>>(),[(15,20,1.),(41,42,10.)]);
    // a file of no solid and nothing else
    assert_eq!(solids(&file_of("AUTOMOTIVE_DESIGN","#1=X();\n")),Ok(vec![]));
}

/// What a file's product structure would say is refused, naming it: an assembly's usage
/// occurrence (Open CASCADE's assembly, and a single part it places), the solid's representation
/// placed twice in its product's shape (two mapped items), solids in the shapes of two products,
/// a placement by a cartesian transformation operator; a solid in no product's shape that a
/// mapped item or a transformation places elsewhere.
#[test]
fn product_structure_refused() {
    let r = refused(ASSEMBLY);
    assert_eq!((r.why,r.entity),(Why::Unsupported,Some(376)),"{r}");
    assert!(parse(ASSEMBLY).get(376).is("NEXT_ASSEMBLY_USAGE_OCCURRENCE"));
    let r = refused(PLACED);
    assert_eq!((r.why,r.entity),(Why::Unsupported,Some(372)),"{r}");
    // the product's shape holds the solid's representation twice, at two places: an instance
    let r = refused(&part(&[("#8","#8=SHAPE_DEFINITION_REPRESENTATION(#7,#30);")],&lines(&mapped(&["#30=SHAPE_REPRESENTATION('',(#31,#36),#20);"],&["#36=MAPPED_ITEM('',#32,#11);"]))));
    assert_eq!((r.why,r.entity),(Why::Unsupported,Some(36)),"{r}");
    assert!(r.msg.contains("placed twice"),"{r}");
    // two products, each with its solid
    let r = refused(&part(&[],&["#40=PRODUCT_DEFINITION('design','',#4,#6);","#41=PRODUCT_DEFINITION_SHAPE('','',#40);",
        "#42=SHAPE_DEFINITION_REPRESENTATION(#41,#43);","#43=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#44),#20);",
        "#44=MANIFOLD_SOLID_BREP('',#16);"]));
    assert_eq!((r.why,r.entity),(Why::Unsupported,Some(40)),"{r}");
    // a mapped item's target a cartesian transformation operator: not read yet
    let r = refused(&part(&[("#8","#8=SHAPE_DEFINITION_REPRESENTATION(#7,#30);")],&lines(&mapped(&["#33=CARTESIAN_TRANSFORMATION_OPERATOR_3D('','',$,#14,#34,#35,2.,#13);"],&[]))));
    assert_eq!((r.why,r.entity),(Why::Unsupported,Some(33)),"{r}");
    // in no product's shape, placed elsewhere by a transformation or a mapped item
    let moved = ["#30=SHAPE_REPRESENTATION('',(#11,#32),#20);","#31=(REPRESENTATION_RELATIONSHIP('','',#10,#30) \
        REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#33) SHAPE_REPRESENTATION_RELATIONSHIP());",
        "#32=AXIS2_PLACEMENT_3D('',#34,#13,#14);","#33=ITEM_DEFINED_TRANSFORMATION('','',#11,#32);","#34=CARTESIAN_POINT('',(5.,0.,0.));"];
    let r = refused(&part(&[("#8","")],&moved));
    assert_eq!((r.why,r.entity),(Why::Unsupported,Some(31)),"{r}");
    let mapped = ["#30=REPRESENTATION_MAP(#11,#10);","#31=MAPPED_ITEM('',#30,#11);","#32=SHAPE_REPRESENTATION('',(#31),#20);"];
    let r = refused(&part(&[("#8","")],&mapped));
    assert_eq!((r.why,r.entity),(Why::Unsupported,Some(31)),"{r}");
    // the same, the solid's representation its product's shape: the product's shape is where it is
    assert_eq!(one_placed(&part(&[],&moved)),None);
    assert_eq!(one_placed(&part(&[],&mapped)),None);
}

/// The product's shape `#30` holding a mapped item `#31` of the solid's representation `#10`: its
/// map `#32` takes the origin `#11` (the identity) onto the target `#33`, at (100, 0, 0) turned a
/// quarter about `z`.
const MAPPED: [&str;8] = ["#30=SHAPE_REPRESENTATION('',(#31),#20);","#31=MAPPED_ITEM('',#32,#33);","#32=REPRESENTATION_MAP(#11,#10);",
    "#33=AXIS2_PLACEMENT_3D('',#34,#13,#35);","#34=CARTESIAN_POINT('',(100.,0.,0.));","#35=DIRECTION('',(0.,1.,0.));",
    "#37=DIRECTION('',(0.,0.,-1.));","#38=CARTESIAN_POINT('',(10.,0.,0.));"];

/// `MAPPED` with lines replaced by their instance's `#n=` and lines added.
fn mapped(replace: &[&str],add: &[&str]) -> Vec<String> {
    let at = |l: &str| l.split('=').next().unwrap().to_string();
    MAPPED.iter().map(|l| replace.iter().find(|r| at(r) == at(l)).unwrap_or(l).to_string())
        .chain(add.iter().map(|s| s.to_string())).collect()
}

/// The one solid's placement in its product's space.
fn one_placed(text: &str) -> Option<context::Placed> {
    let s = solids(text).unwrap_or_else(|r| panic!("{r}"));
    assert_eq!(s.len(),1);
    s[0].placed
}

fn moved_like(p: Option<context::Placed>,by: u32,r: [[f64;3];3],t: [f64;3]) {
    let p = p.expect("the solid should be placed");
    assert_eq!(p.by,by,"{p:?}");
    assert!((0..3).all(|i| (0..3).all(|j| (p.motion.r[i][j]-r[i][j]).abs() <= 1e-15) && (p.motion.t[i]-t[i]).abs() <= 1e-12),"{p:?}");
}

/// **An axis placement among a representation's items places nothing** (ISO 10303-43: a
/// representation's items share its context's space; an `AXIS2_PLACEMENT_3D` there is a datum, as
/// most writers put the part's frame, and moves its siblings only where a mapping or a
/// transformation uses it): off the origin or turned, in the solid's representation or in a
/// construction representation tied to it, the solid is read where it is.
#[test]
fn a_placement_item_places_nothing() {
    assert_eq!(one_placed(&part(&[("#12","#12=CARTESIAN_POINT('',(0.,0.,1.));")],&[])),None);
    assert_eq!(one_placed(&part(&[("#14","#14=DIRECTION('',(0.,1.,0.));")],&[])),None);
    let text = part(&[],&["#30=CONSTRUCTIVE_GEOMETRY_REPRESENTATION('supplemental geometry',(#31),#20);",
        "#31=AXIS2_PLACEMENT_3D('DRF_ABC',#32,#13,#33);","#32=CARTESIAN_POINT('',(3.,4.,5.));","#33=DIRECTION('',(0.,1.,0.));",
        "#34=CONSTRUCTIVE_GEOMETRY_REPRESENTATION_RELATIONSHIP('supplemental geometry','',#10,#30);"]);
    assert_eq!(one_placed(&text),None);
    let s = context::shapes(&parse(&text)).unwrap();
    assert_eq!(s.beside.iter().map(|b| (b.entity,b.what)).collect::<Vec<_>>(),[(31,"construction geometry")]);
}

/// **A solid placed is the solid moved**: a mapped item in its product's shape takes the mapped
/// representation's origin onto its target (ISO 10303-43's `mapped_item`), a representation
/// relationship with an item-defined transformation takes `rep_1`'s item onto `rep_2`'s (rep_1 the
/// placed, as Open CASCADE's assemblies write it), each a rigid motion of two right-handed axes.
/// The origin is read in the mapped representation's units, the target in the representation
/// holding the mapped item: a part in centimetres mapped into a product in millimetres is
/// placed, not scaled. A mapped item in a draughting model presents the shape and places
/// nothing.
#[test]
fn a_mapped_solid_is_moved() {
    let shape = ("#8","#8=SHAPE_DEFINITION_REPRESENTATION(#7,#30);");
    let quarter = [[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]];
    let s = solids(&part(&[shape],&MAPPED)).unwrap();
    assert_eq!((s[0].solid,&s[0].name[..],s[0].context.entity),(15,"name one",20));
    moved_like(s[0].placed,31,quarter,[100.,0.,0.]);
    // the origin off the identity: what stood on it stands on the target
    moved_like(one_placed(&part(&[shape],&lines(&mapped(&["#32=REPRESENTATION_MAP(#36,#10);"],
        &["#36=AXIS2_PLACEMENT_3D('',#38,$,$);"])))),31,quarter,[100.,-10.,0.]);
    // the mapped representation in centimetres: its origin 1 cm along x
    moved_like(one_placed(&part(&[shape,("#10","#10=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#11,#15),#40);")],
        &lines(&mapped(&["#32=REPRESENTATION_MAP(#36,#10);"],&["#36=AXIS2_PLACEMENT_3D('',#39,$,$);","#39=CARTESIAN_POINT('',(1.,0.,0.));",
        "#40=(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#41,#22)) REPRESENTATION_CONTEXT('',''));",
        "#41=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.CENTI.,.METRE.));"])))),31,quarter,[100.,-10.,0.]);
    // turned upside down about x: a right-handed frame whose axis is -z and reference x
    moved_like(one_placed(&part(&[shape],&lines(&mapped(&["#33=AXIS2_PLACEMENT_3D('',#34,#37,#14);"],&[])))),31,
        [[1.,0.,0.],[0.,-1.,0.],[0.,0.,-1.]],[100.,0.,0.]);
    // the identity mapped: nothing moves
    assert_eq!(one_placed(&part(&[shape],&lines(&mapped(&["#33=AXIS2_PLACEMENT_3D('',#12,$,$);"],&[])))),None);
    // by a representation relationship with a transformation, the product's shape its rep_2
    let rrwt = ["#30=SHAPE_REPRESENTATION('',(#33),#20);","#31=(REPRESENTATION_RELATIONSHIP('','',#10,#30) \
        REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#36) SHAPE_REPRESENTATION_RELATIONSHIP());",
        "#33=AXIS2_PLACEMENT_3D('',#34,#13,#35);","#34=CARTESIAN_POINT('',(100.,0.,0.));","#35=DIRECTION('',(0.,1.,0.));",
        "#36=ITEM_DEFINED_TRANSFORMATION('','',#11,#33);"];
    moved_like(one_placed(&part(&[shape],&rrwt)),31,quarter,[100.,0.,0.]);
    // through a shape representation relationship and then the mapped item
    moved_like(one_placed(&part(&[("#8","#8=SHAPE_DEFINITION_REPRESENTATION(#7,#50);")],&lines(&mapped(&[],
        &["#50=SHAPE_REPRESENTATION('',(#11),#20);","#51=SHAPE_REPRESENTATION_RELATIONSHIP('','',#30,#50);"])))),31,quarter,[100.,0.,0.]);
    // a mapped item in a draughting model presents the shape and places nothing
    let text = part(&[],&lines(&mapped(&["#30=(DRAUGHTING_MODEL() REPRESENTATION('',(#31),#20));"],&[])));
    assert_eq!(one_placed(&text),None);
    assert_eq!(context::shapes(&parse(&text)).unwrap().beside.iter().map(|b| (b.entity,b.what)).collect::<Vec<_>>(),
        [(30,"annotation (a draughting model)")]);
}

fn lines(v: &[String]) -> Vec<&str> { v.iter().map(String::as_str).collect() }

/// Open CASCADE's drilled block (`drilled_ap214.step`, and in inches `drilled_inch.step`) mapped
/// into a product's shape in millimetres at (100, 0, 0) turned a quarter about `z`: read, every
/// vertex is the unmoved reading's moved so, and its volume is the block's.
#[test]
fn a_mapped_solid_reads_moved() {
    use gcs_core::brep::step_read::read;
    let mapped = |text: &str,context: &str,more: &str| -> String {
        let at = text.rfind("ENDSEC;").unwrap();
        format!("{}#500 = SHAPE_REPRESENTATION('',(#501),{context});\n#501 = MAPPED_ITEM('',#502,#503);\n\
            #502 = REPRESENTATION_MAP(#11,#10);\n#503 = AXIS2_PLACEMENT_3D('',#504,#505,#506);\n#504 = CARTESIAN_POINT('',(100.,0.,0.));\n\
            #505 = DIRECTION('',(0.,0.,1.));\n#506 = DIRECTION('',(0.,1.,0.));\n{more}{}",
            &text[..at].replace("#3 = SHAPE_DEFINITION_REPRESENTATION(#4,#10);","#3 = SHAPE_DEFINITION_REPRESENTATION(#4,#500);"),&text[at..])
    };
    let mm = "#510 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#511,#434,#435)) REPRESENTATION_CONTEXT('','') );\n\
        #511 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n";
    for (text,moved) in [(DRILLED_AP214,mapped(DRILLED_AP214,"#429","")),(DRILLED_INCH,mapped(DRILLED_INCH,"#510",mm))] {
        let was = read(text).unwrap_or_else(|r| panic!("{r}"));
        let now = read(&moved).unwrap_or_else(|r| panic!("{r}"));
        assert_eq!((was.len(),now.len()),(1,1));
        let (a,b) = (&was[0].brep,&now[0].brep);
        assert_eq!(a.vertices.len(),b.vertices.len());
        for (u,v) in a.vertices.iter().zip(&b.vertices) {
            let want = [100.-u.p[1],u.p[0],u.p[2]];
            assert!((0..3).all(|i| (v.p[i]-want[i]).abs() <= 1e-9),"{:?} moved to {:?}",u.p,v.p);
        }
        let (va,vb) = (gcs_core::brep::props::volume(a),gcs_core::brep::props::volume(b));
        assert!((va-vb).abs() <= 1e-9*va && (va-(40.*30.*20.-std::f64::consts::PI*36.*20.)).abs() <= 1e-6*va,"{va} {vb}");
        assert_eq!(now[0].context,was[0].context,"the solid's context is its own representation's");
    }
}

/// **A solid with other geometry beside it is still that solid**, and what was left beside it is
/// said: a surface model (open, and closed round a volume), a set of curves in a wireframe
/// representation, construction geometry and annotation, each by its entity, in the file's order;
/// a shell said with its surface model, not again. A file holding no solid but such things is
/// refused as before.
#[test]
fn beside_the_solid_is_said() {
    let text = part(&[],&["#30=MANIFOLD_SURFACE_SHAPE_REPRESENTATION('',(#11,#31,#33),#20);","#31=SHELL_BASED_SURFACE_MODEL('Fill.1',(#32));",
        "#32=OPEN_SHELL('',());","#33=SHELL_BASED_SURFACE_MODEL('',(#16));","#34=SHAPE_REPRESENTATION_RELATIONSHIP('','',#10,#30);",
        "#35=GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION('',(#36),#20);","#36=GEOMETRIC_CURVE_SET('',());",
        "#37=SHAPE_REPRESENTATION('',(#32),#20);","#38=CONSTRUCTIVE_GEOMETRY_REPRESENTATION('',(#39),#20);","#39=LINE('',#12,#40);",
        "#40=VECTOR('',#13,1.);","#41=(DRAUGHTING_MODEL() REPRESENTATION('',(),#20));"]);
    let s = context::shapes(&parse(&text)).unwrap();
    assert_eq!(s.solids.iter().map(|f| (f.solid,f.placed)).collect::<Vec<_>>(),[(15,None)]);
    assert_eq!(s.beside.iter().map(|b| (b.entity,&b.kind[..],b.what)).collect::<Vec<_>>(),[
        (31,"SHELL_BASED_SURFACE_MODEL","a shell-based surface model"),
        (33,"SHELL_BASED_SURFACE_MODEL","a shell-based surface model closed round a volume"),
        (36,"GEOMETRIC_CURVE_SET","a set of curves"),(39,"LINE","construction geometry"),
        (41,"DRAUGHTING_MODEL+REPRESENTATION","annotation (a draughting model)")]);
    let r = refused(SHELL);
    assert_eq!((r.why,r.entity),(Why::Unsupported,Some(15)),"{r}");
}

/// A file that holds no solid but something else is refused saying what, naming the entity; a
/// solid in a representation of another kind, or in none, is refused.
#[test]
fn what_is_no_solid_refused() {
    let r = refused(SHELL);
    assert_eq!((r.why,r.entity),(Why::Unsupported,Some(15)),"{r}");
    assert!(r.msg.contains("SHELL_BASED_SURFACE_MODEL"),"{r}");
    for (replace,entity,what) in [
        (("#15","#15=FACETED_BREP('',#16);"),15,"FACETED_BREP"),
        (("#15","#15=SHELL_BASED_SURFACE_MODEL('',(#16));"),15,"SHELL_BASED_SURFACE_MODEL"),
        (("#15","#15=TESSELLATED_SOLID('',(),$);"),15,"TESSELLATED_SOLID"),
        (("#15",""),16,"CLOSED_SHELL"),
    ] {
        let mut text = part(&[replace],&[]);
        if replace.1.is_empty() { text = text.replace("(#11,#15)","(#11)"); }
        let r = refused(&text);
        assert_eq!((r.why,r.entity),(Why::Unsupported,Some(entity)),"{what}: {r}");
        assert!(r.msg.contains(what),"{what}: {r}");
    }
    let r = refused(&part(&[("#10","#10=MANIFOLD_SURFACE_SHAPE_REPRESENTATION('',(#11,#15),#20);")],&[]));
    assert_eq!((r.why,r.entity),(Why::Unsupported,Some(10)),"{r}");
    let r = refused(&part(&[],&["#30=MANIFOLD_SOLID_BREP('',#16);"]));
    assert_eq!((r.why,r.entity),(Why::Unsupported,Some(30)),"{r}");
    // beside the solid, a surface model or tessellated geometry is not read (`beside_the_solid_is_said`)
    assert!(solids(&part(&[],&["#30=SHELL_BASED_SURFACE_MODEL('',(#16));"])).is_ok());
    assert!(solids(&part(&[],&["#30=TESSELLATED_SHAPE_REPRESENTATION('',(),#20);"])).is_ok());
}

/// The kernel's own files parse as before (`step_check` holds the rest).
#[test]
fn kernel_files_parse() {
    let text = gcs_core::brep::step::write(&gcs_core::brep::build::block([0.,0.,0.],[3.,2.,1.]).unwrap(),"cube",1e-7,
        gcs_core::brep::step::Fit::Exact).unwrap();
    let f = parse(&text);
    assert_eq!(context::schema(&f),Ok(Schema::Ap214));
    let s = context::solids(&f).unwrap();
    assert_eq!((s.len(),s[0].context.length,s[0].context.angle),(1,1.,1.));
}

/// The corpus of files Open CASCADE wrote (`/home/user/step-corpus/cadgenbench`, outside the
/// repository): each file's schema, units, uncertainty and solids, and its parse's time.
#[test]
#[ignore = "reads a corpus outside the repository"]
fn corpus_files() {
    let dir = std::path::Path::new("/home/user/step-corpus/cadgenbench");
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    let mut ids: Vec<_> = entries.flatten().map(|e| e.path()).collect();
    ids.sort();
    for d in ids {
        let Ok(text) = std::fs::read_to_string(d.join("input.step")) else { continue };
        let clock = std::time::Instant::now();
        let f = context::parse(&text).unwrap();
        let t = clock.elapsed();
        let found = context::solids(&f);
        println!("{}\t{:?}\t{:?}\t{:?}",d.display(),f.header.schema_names(),found.as_ref().map(|s| s.iter().map(|x| (x.solid,x.context)).collect::<Vec<_>>()),t);
    }
}
