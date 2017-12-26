using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.ComponentModel.Composition;

using VisualComponents.Create3D;
using VisualComponents.UX.Ribbon;
using VisualComponents.UX.Shared;
using Caliburn.Micro;
using System.Windows.Input;

// Diese Datei fügt die Anbindung einer Ribbon in Visual Components.
// In der Ribbon kann die Kamerageschwindigkeit ueber ein Numfield geaendert.
// Für die Doku sind Verweise in diese Datei als R beschrieben.

namespace Debugger
{
    // Hier der Teil von MyCameraPlugin, um die Generierung des Ribbons innerhalb von M aufzurufen
    partial class DebugInterface
    {
        // Generierung des Ribbons (oder Registrierung.. wie auch immer..)

        private void registerUXSite()
        {
            // in IUXConfiguration wird der eigene Ribbon angemeldet
            IUXConfiguration uxConfig = IoC.Get<IUXConfiguration>();
            // mit ICommandRegistry kann man auf Actions zugreifen
            ICommandRegistry cmd = IoC.Get<ICommandRegistry>();

            // Erstellung des Ribbons (R1)
            var ribbon = new UXSiteSetup
            {
                // TabId muss suffix sein !
                UXSiteIdPath = "DebugRibbonTabId",
                UXSiteType = UXSiteType.Tab

            };
            // Anmeldung des Ribbons
            uxConfig.RegisterSite(ribbon);
            
            /*
            // Erstellung der Ribbongruppe (klappt irgendwie noch nicht so ganz wie erwuenscht) (R2)
            var ribbonGroup = new UXSiteSetup
            {
                UXSiteIdPath = "DebugRibbonTabId" + "/" + "DebugBasicRibbonGroup",
                UXSiteType = UXSiteType.RibbonGroup
            };
            // Anmeldung der Ribbongruppe
            uxConfig.RegisterSite(ribbonGroup);

            var ribbonGroup2 = new UXSiteSetup
            {
                UXSiteIdPath = "DebugRibbonTabId"+"/"+"DebugCheckRibbonGroup",
                UXSiteType = UXSiteType.RibbonGroup
            };
            // Anmeldung der Ribbongruppe
            uxConfig.RegisterSite(ribbonGroup2);

            var ribbonGroup3 = new UXSiteSetup
            {
                UXSiteIdPath = "DebugRibbonTabId"+"/"+"DebugTextRibbonGroup",
                UXSiteType = UXSiteType.RibbonGroup
            };
            // Anmeldung der Ribbongruppe
            uxConfig.RegisterSite(ribbonGroup3);

            var ribbonGroup4 = new UXSiteSetup
            {
                UXSiteIdPath = "DebugRibbonTabId"+"/"+"DebugNumRibbonGroup",
                UXSiteType = UXSiteType.RibbonGroup
            };
            // Anmeldung der Ribbongruppe
            uxConfig.RegisterSite(ribbonGroup4);
            */

            for (int i = 0; i < IoC.Get<IDebugCall>().size; i++)
            {

                CallbuttonItem CallerActionItem = new CallbuttonItem(i);

                var ButtonRibbonItem = new UXSiteSetup
                {
                    UXSiteIdPath = "DebugRibbonTabId/DebugBasicRibbonGroup",
                    EntryId = CallerActionItem.Id                  
                };
                
                cmd.RegisterActionItem(CallerActionItem, ButtonRibbonItem);

                CheckbuttonItem CheckerActionItem = new CheckbuttonItem(i);

                var CheckButtonRibbonItem = new UXSiteSetup
                {
                    UXSiteIdPath = "DebugRibbonTabId/DebugCheckRibbonGroup",
                    EntryId = CheckerActionItem.Id
                };

                cmd.RegisterActionItem(CheckerActionItem, CheckButtonRibbonItem);

                TexteditorItem TextActionItem = new TexteditorItem(i);

                var TexteditorRibbonItem = new UXSiteSetup
                {
                    UXSiteIdPath = "DebugRibbonTabId/DebugTextRibbonGroup",
                    EntryId = TextActionItem.Id
                };

                cmd.RegisterActionItem(TextActionItem, TexteditorRibbonItem);

                NumeditorItem NumActionItem = new NumeditorItem(i);

                var NumeditorRibbonItem = new UXSiteSetup
                {
                    UXSiteIdPath = "DebugRibbonTabId/DebugNumRibbonGroup",
                    EntryId = NumActionItem.Id
                };

                cmd.RegisterActionItem(NumActionItem, NumeditorRibbonItem);

            }
        }
        
    }

    // Die Klasse bildet die eigene Ribbon mit Namen (R1)
    [Export(typeof(IRibbonTab))]
    class CameraRibbon : RibbonTabBase
    {

        // Wurde übernommen. Genaue Funktion unbekannt.
        // Grob wird ein ILocalizationService und die IRibbonGroup Liste genutzt um daraus mit dem super-Contructor vermutlich eine sinnvolle Ribbon zu generieren...
        [ImportingConstructor]
        public CameraRibbon([Import] ILocalizationService localizationService, [ImportMany] IEnumerable<IRibbonGroup> groups) : base(localizationService, groups)
        {
        }

        // HEADER ist der Name, den man dann oben auch sieht zum Auswählen des Ribbons
        public override string Header
        {
            get
            {
                return "DEBUG";
            }
        }

        // Die ID, über die mit dem UXSiteSetup diese Ribbon generiert/registriert wird
        // ACHTUNG TabId muss der Suffix im Id sein!!!
        public override string Id
        {
            get
            {
                return "DebugRibbonTabId";
            }
        }

        // Ich vermute es aendert die Reihenfolge...
        public override double Order
        {
            get
            {
                return 10.0;
            }
        }

        // Habe nicht herausgekriegt wo der Name zu sehen ist...
        public override string Name
        {
            get
            {
                return "DebugRibbon";
            }
        }

        // Ebenfalls keinen Plan
        public override string ContextualTabGroupKey
        {
            get
            {
                return "";
            }
        }
    }
    /*
    // Ribbon Group (R2)
    [Export(typeof(IRibbonGroup))]
    class RibbonGroup : RibbonGroupBase
    {
        // So wie oben bei (R1)
        [ImportingConstructor]
        public RibbonGroup([Import] ILocalizationService localizationService) : base(localizationService)
        {
        }

        // Haette erwartet, dass das der Name unter der Group ist. Ist es aber irgendwie nicht...
        public override string Header
        {
            get
            {
                return "Base Settings";
            }
        }

        // Nicht getestet, aber die Id wuerde ich immer beim suffix RibbonGroup belassen (Siehe Problematik bei (R1)). Die Id wird vom UXSiteSetup genutzt.
        public override string Id
        {
            get
            {
                return "DebugBasicRibbonGroup";
            }
        }

        // Ich vermute mal, man kann ein Icon zeigen lassen. Man muss einen Pfad zur Ressource angeben (vermutlich auch als Ressource in der .dll?)
        public override string Icon
        {
            get
            {
                return "";
            }
        }


    }

    [Export(typeof(IRibbonGroup))]
    class RibbonGroup2 : RibbonGroupBase
    {
        // So wie oben bei (R1)
        [ImportingConstructor]
        public RibbonGroup2([Import] ILocalizationService localizationService) : base(localizationService)
        {
        }

        // Haette erwartet, dass das der Name unter der Group ist. Ist es aber irgendwie nicht...
        public override string Header
        {
            get
            {
                return "Base Settings";
            }
        }

        // Nicht getestet, aber die Id wuerde ich immer beim suffix RibbonGroup belassen (Siehe Problematik bei (R1)). Die Id wird vom UXSiteSetup genutzt.
        public override string Id
        {
            get
            {
                return "DebugCheckRibbonGroup";
            }
        }

        // Ich vermute mal, man kann ein Icon zeigen lassen. Man muss einen Pfad zur Ressource angeben (vermutlich auch als Ressource in der .dll?)
        public override string Icon
        {
            get
            {
                return "";
            }
        }


    }


    [Export(typeof(IRibbonGroup))]
    class RibbonGroup3 : RibbonGroupBase
    {
        // So wie oben bei (R1)
        [ImportingConstructor]
        public RibbonGroup3([Import] ILocalizationService localizationService) : base(localizationService)
        {
        }

        // Haette erwartet, dass das der Name unter der Group ist. Ist es aber irgendwie nicht...
        public override string Header
        {
            get
            {
                return "Base Settings";
            }
        }

        // Nicht getestet, aber die Id wuerde ich immer beim suffix RibbonGroup belassen (Siehe Problematik bei (R1)). Die Id wird vom UXSiteSetup genutzt.
        public override string Id
        {
            get
            {
                return "DebugTextRibbonGroup";
            }
        }

        // Ich vermute mal, man kann ein Icon zeigen lassen. Man muss einen Pfad zur Ressource angeben (vermutlich auch als Ressource in der .dll?)
        public override string Icon
        {
            get
            {
                return "";
            }
        }


    }


    [Export(typeof(IRibbonGroup))]
    class RibbonGroup4 : RibbonGroupBase
    {
        // So wie oben bei (R1)
        [ImportingConstructor]
        public RibbonGroup4([Import] ILocalizationService localizationService) : base(localizationService)
        {
        }

        // Haette erwartet, dass das der Name unter der Group ist. Ist es aber irgendwie nicht...
        public override string Header
        {
            get
            {
                return "Base Settings";
            }
        }

        // Nicht getestet, aber die Id wuerde ich immer beim suffix RibbonGroup belassen (Siehe Problematik bei (R1)). Die Id wird vom UXSiteSetup genutzt.
        public override string Id
        {
            get
            {
                return "DebugNumRibbonGroup";
            }
        }

        // Ich vermute mal, man kann ein Icon zeigen lassen. Man muss einen Pfad zur Ressource angeben (vermutlich auch als Ressource in der .dll?)
        public override string Icon
        {
            get
            {
                return "";
            }
        }


    }*/

    /// <summary>
    /// //////////////////////////////////////////////////////////////////////////
    /// </summary>

    [Export(typeof(ActionItem))]
    class CallbuttonItem : ActionItem
    {
        private int _invokerIndex;
        // Kontruktor mit Default Text vor der Eingabe und der ID, mit dem dieses Item registriert werden kann
        public CallbuttonItem() : base("CallbuttonId", "Call", "", null)
        {
            _invokerIndex = 0;
        }

        public CallbuttonItem(int index) : base("CallbuttonId" + index.ToString(), "Call " + (index + 1).ToString(), "", null) {
            _invokerIndex = index;
            
        }
        
        public override void Execute()
        {
            IoC.Get<IDebugCall>().DebugCall[_invokerIndex]?.Invoke();
        }

    }

    [Export(typeof(ActionItem))]
    class CheckbuttonItem : ActionItem, ICheckBoxToolCommand
    {
        private int _invokerIndex;
        // Kontruktor mit Default Text vor der Eingabe und der ID, mit dem dieses Item registriert werden kann
        public CheckbuttonItem() : base("CheckbuttonId", "Check", "", null)
        {
            _invokerIndex = 0;
        }

        public CheckbuttonItem(int index) : base("CheckbuttonId" + index.ToString(), "Check " + (index + 1).ToString(), "", null, MenuTools.CheckBoxTool)
        {
            _invokerIndex = index;

        }
        
        public bool IsChecked
        {
            get
            {
                return IoC.Get<IDebugCall>().CheckValue[_invokerIndex];
            }

            set
            {
                IoC.Get<IDebugCall>().CheckValue[_invokerIndex] = value;
                IoC.Get<IDebugCall>().DebugCheck[_invokerIndex]?.Invoke();
            }
        }

    }

    [Export(typeof(ActionItem))]
    class TexteditorItem : ActionItem, ITextEditorTool
    {
        private int _invokerIndex;
        // Kontruktor mit Default Text vor der Eingabe und der ID, mit dem dieses Item registriert werden kann
        public TexteditorItem() : base("TexteditorId", "Textfield", "", null)
        {
            _invokerIndex = 0;
        }

        public TexteditorItem(int index) : base("TexteditorId" + index.ToString(), "Textfield " + (index + 1).ToString(), "", null, MenuTools.TextEditorTool)
        {
            _invokerIndex = index;

        }
                
        public bool IsReadOnly
        {
            get
            {
                return false;
            }

            set
            {
            }
        }

        private object _last;
        private object _Text;
        public object Text
        {
            get
            {
                return _Text;
            }

            set
            {
                _last = _Text;
                _Text = value;
                if (_Text == null) {
                    IoC.Get<IDebugCall>().TextValue[_invokerIndex] = "";
                    if (_last != null) {
                        IoC.Get<IDebugCall>().DebugText[_invokerIndex]?.Invoke();
                    }
                } else {
                    if (_last == null || !_Text.Equals(_last))
                    {
                        IoC.Get<IDebugCall>().TextValue[_invokerIndex] = value.ToString();
                        IoC.Get<IDebugCall>().DebugText[_invokerIndex]?.Invoke();
                    }
                }
                
            }
        }

        public double EditAreaWidth
        {
            get
            {
                return 100;
            }

            set
            {
                
            }
        }
        
        public void OnPreviewKeyDown(string text, System.Windows.Input.KeyEventArgs e)
        {
            throw new NotImplementedException();
        }
    }

    [Export(typeof(ActionItem))]
    class NumeditorItem : ActionItem, INumericTextEditorTool
    {
        private int _invokerIndex;
        // Kontruktor mit Default Text vor der Eingabe und der ID, mit dem dieses Item registriert werden kann
        public NumeditorItem() : base("NumeditorId", "Numfield", "", null)
        {
            _invokerIndex = 0;
        }

        public NumeditorItem(int index) : base("NumeditorId" + index.ToString(), "Numfield " + (index + 1).ToString(), "", null, MenuTools.NumericTextEditorTool)
        {
            _invokerIndex = index;

        }

        private void toggle()
        {
            IoC.Get<IMessageService>().AppendMessage("Numeditor " + _invokerIndex.ToString() + " " + _Text.ToString(), MessageLevel.Warning);
        }

        public bool IsReadOnly
        {
            get
            {
                return false;
            }

            set
            {
            }
        }

        private object _last;
        private object _Text;
        public object Text
        {
            get
            {
                return _Text;
            }

            set
            {
                _last = _Text;
                _Text = value;
                if (_Text == null)
                {
                    IoC.Get<IDebugCall>().NumValue[_invokerIndex] = 0;
                    if (_last != null)
                    {
                        IoC.Get<IDebugCall>().DebugNum[_invokerIndex]?.Invoke();
                    }
                }
                else
                {
                    if (_last == null || !_Text.Equals(_last))
                    {
                        IoC.Get<IDebugCall>().NumValue[_invokerIndex] = Convert.ToDouble(value.ToString());
                        IoC.Get<IDebugCall>().DebugNum[_invokerIndex]?.Invoke();
                    }
                }

            }
        }

        public double EditAreaWidth
        {
            get
            {
                return 100;
            }

            set
            {

            }
        }

        public string UnitSuffix
        {
            get
            {
                return "";
            }
        }

        public void OnPreviewKeyDown(string text, System.Windows.Input.KeyEventArgs e)
        {
            throw new NotImplementedException();
        }
    }


}