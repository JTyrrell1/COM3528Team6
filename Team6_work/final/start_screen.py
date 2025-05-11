import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from pathlib import Path
from PIL import Image, ImageTk
import fitz  # PyMuPDF
import time
import psutil
import subprocess
from multiprocessing import Process, Manager

from Team6_work.image_description import ImageDescriber
from Team6_work.vapi_therapist import Vapi_TheRapist
from Team6_work.final.main import create_app  # <-- app factory
from Team6_work.final.miro_emotions import run_miro  # <-- make sure this uses run_miro(shared_state)

def run_vapi_in_process(image_description, patient_history):
    from Team6_work.vapi_therapist import Vapi_TheRapist
    vapi = Vapi_TheRapist(image_description, patient_history)
    vapi.create_and_start_assistant()

class ReminiscenceTherapyGUI(ttk.Frame):
    def __init__(self, root, patient_info, theme_path="Team6_work/Azure-ttk-theme-main/azure.tcl"):
        super().__init__(root, padding=20)
        self.root = root

        self.root.tk.call("source", theme_path)
        self.root.tk.call("set_theme", "light")
        self.root.title("Reminiscence Therapy")

        try:
            self.root.state("zoomed")
        except tk.TclError:
            try:
                self.root.attributes("-zoomed", True)
            except tk.TclError:
                self.root.geometry(f"{self.root.winfo_screenwidth()}x{self.root.winfo_screenheight()}+0+0")

        self.root.minsize(800, 600)
        self.pack(fill="both", expand=True)

        self.patient_name = str(patient_info.get("name", "Unknown"))
        self.patient_age = str(patient_info.get("age", "Unknown"))
        self.history_file_path = patient_info.get("history_path")
        self.history_text = None

        self.image_file_path = None
        self.image_type = tk.StringVar(value="personal")

        self._build_image_panel()
        self._build_info_panel()

        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=2)
        self.rowconfigure(0, weight=1)

        if self.history_file_path:
            self._preview_pdf(self.history_file_path)
            self.history_text = self._extract_text_from_pdf(self.history_file_path)

        self.vapi_proc = None
        self.api_proc = None
        self.miro_proc = None


    def _build_image_panel(self):
        panel = ttk.LabelFrame(self, text="Reminiscence Image", padding=12)
        panel.grid(row=0, column=0, sticky="nsew", padx=(0, 20))

        self.image_display = ttk.Label(panel, text="No image uploaded", anchor="center", justify="center")
        self.image_display.pack(expand=True, fill="both")

        ttk.Button(panel, text="Upload Image", command=self._upload_image).pack(pady=8)

        desc_frame = ttk.LabelFrame(panel, text="Image Description", padding=8)
        desc_frame.pack(fill="x", pady=(10, 0))

        self.image_description_widget = tk.Text(desc_frame, height=5, wrap="word", borderwidth=0, highlightthickness=0)
        self.image_description_widget.pack(fill="both", expand=True)

    def _build_info_panel(self):
        panel = ttk.Frame(self)
        panel.grid(row=0, column=1, sticky="nsew")

        info_frame = ttk.LabelFrame(panel, text="Patient Info", padding=12)
        info_frame.pack(fill="x")
        ttk.Label(info_frame, text=f"Name: {self.patient_name}").pack(anchor="w")
        ttk.Label(info_frame, text=f"Age: {self.patient_age}").pack(anchor="w")

        self.history_label = ttk.Label(info_frame, text="History: none uploaded", foreground="grey")
        self.history_label.pack(anchor="w", pady=(5, 0))
        ttk.Button(info_frame, text="Upload History PDF", command=self._upload_history_pdf).pack(anchor="e", pady=5)

        self.history_preview_frame = ttk.LabelFrame(panel, text="History Preview", padding=6)
        self.history_preview_frame.pack(fill="x", pady=10)

        self.preview_image_label = ttk.Label(self.history_preview_frame, text="No preview available", anchor="center", justify="center")
        self.preview_image_label.pack(expand=True, fill="both")

        img_type_frame = ttk.LabelFrame(panel, text="Image Type", padding=12)
        img_type_frame.pack(fill="x", pady=10)
        ttk.Radiobutton(img_type_frame, text="Personal", variable=self.image_type, value="personal").pack(anchor="w")
        ttk.Radiobutton(img_type_frame, text="Generic", variable=self.image_type, value="generic").pack(anchor="w")

        ttk.Button(panel, text="Start Server", command=self._start_server).pack(fill="x", pady=(10, 5), ipady=8)
        ttk.Button(panel, text="Start Therapy", command=self._start_therapy_session).pack(fill="x", pady=(0, 10), ipady=10)

    def _upload_image(self):
        file_path = filedialog.askopenfilename(filetypes=[("Image Files", "*.png *.jpg *.jpeg")])
        if file_path:
            self.image_file_path = file_path
            image = Image.open(file_path)
            image.thumbnail((600, 600))
            photo = ImageTk.PhotoImage(image)
            self.image_display.configure(image=photo, text="")
            self.image_display.image = photo

            describer = ImageDescriber(image_path=file_path)
            self.image_description_widget.delete("1.0", "end")
            self.image_description_widget.insert("1.0", describer.response.output_text)

    def _upload_history_pdf(self):
        file_path = filedialog.askopenfilename(filetypes=[("PDF Files", "*.pdf")])
        if file_path:
            self.history_file_path = file_path
            self._preview_pdf(file_path)
            self.history_text = self._extract_text_from_pdf(file_path)

    def _preview_pdf(self, pdf_path):
        self.history_label.config(text=f"History: {Path(pdf_path).name}")
        doc = fitz.open(pdf_path)
        page = doc.load_page(0)
        pix = page.get_pixmap()
        mode = "RGBA" if pix.alpha else "RGB"
        img = Image.frombytes(mode, (pix.width, pix.height), pix.samples)
        img.thumbnail((500, 400))
        photo = ImageTk.PhotoImage(img)
        self.preview_image_label.configure(image=photo, text="")
        self.preview_image_label.image = photo
        doc.close()

    def _extract_text_from_pdf(self, pdf_path):
        doc = fitz.open(pdf_path)
        text = doc[0].get_text()
        doc.close()
        return text.strip()
    
    def _show_therapy_view(self):
        self.pack_forget()
        self.therapy_frame = TherapySessionFrame(self.root, self.patient_name, self.shared_state, self._return_to_main_view)
        self.therapy_frame.pack(fill="both", expand=True)

    def _return_to_main_view(self):
        self._end_all_processes()  # ← Kill subprocesses
        self.therapy_frame.pack_forget()
        self.pack(fill="both", expand=True)

    def _show_url_dialog(self, url):
        window = tk.Toplevel(self)
        window.title("Webhook URL")
        window.geometry("500x120")
        ttk.Label(window, text="Webhook is now public at:").pack(pady=(10, 0))

        entry = ttk.Entry(window, width=60)
        entry.insert(0, url)
        entry.pack(pady=5)
        entry.select_range(0, 'end')

        def copy_to_clipboard():
            self.root.clipboard_clear()
            self.root.clipboard_append(url)
            messagebox.showinfo("Copied", f"Copied to clipboard:\n{url}")

        ttk.Button(window, text="Copy to Clipboard", command=copy_to_clipboard).pack(pady=5)

    def _free_port(self, port=8000):
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                connections = proc.connections(kind='inet')
                for conn in connections:
                    if conn.laddr.port == port:
                        proc.kill()
                        print(f"Killed process {proc.pid} using port {port}")
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue

    def _start_server(self):
        try:
            self._free_port(8000)

            # Create shared state
            self.manager = Manager()
            self.shared_state = self.manager.dict(current_mode="idle", agent_speaking=False)

            # Start FastAPI via uvicorn.run
            from uvicorn import run
            app = create_app(self.shared_state)
            self.api_proc = Process(target=run, kwargs={"app": app, "host": "0.0.0.0", "port": 8000}, daemon=True)
            self.api_proc.start()

            # Start ngrok via subprocess
            self.ngrok_proc = subprocess.Popen([
                "bash", "-c", "cd ~ && ./ngrok http --domain=eminent-sought-beetle.ngrok-free.app 8000"
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


            # Start MiRo behavior process
            self.miro_proc = Process(target=run_miro, args=(self.shared_state,), daemon=True)
            self.miro_proc.start()

            time.sleep(2)
            self._show_url_dialog("https://eminent-sought-beetle.ngrok-free.app")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to start server:\n{e}")

    def _start_therapy_session(self):
        image_description = self.image_description_widget.get("1.0", "end").strip()

        if image_description and self.history_text:
            self.vapi_proc = Process(
                target=run_vapi_in_process,
                args=(image_description, self.history_text),
                daemon=True
            )
            self.vapi_proc.start()
            self._show_therapy_view()
        else:
            messagebox.showwarning("Missing Information", "Please upload both an image and a history PDF before starting.")

    def _end_all_processes(self):
        print("[MainGUI] Cleaning up processes...")

        # vapi
        if self.vapi_proc and self.vapi_proc.is_alive():
            self.vapi_proc.terminate()
            self.vapi_proc.join()
            print("[MainGUI] Vapi process terminated.")

        # FastAPI
        if self.api_proc and self.api_proc.is_alive():
            self.api_proc.terminate()
            self.api_proc.join()
            print("[MainGUI] FastAPI terminated.")

        # MiRo
        if self.miro_proc and self.miro_proc.is_alive():
            self.miro_proc.terminate()
            self.miro_proc.join()
            print("[MainGUI] MiRo process terminated.")

        # ngrok
        if hasattr(self, 'ngrok_proc') and self.ngrok_proc and self.ngrok_proc.poll() is None:
            self.ngrok_proc.terminate()
            print("[MainGUI] ngrok terminated.")

    def _on_app_exit(self):
        self._end_all_processes()
        self.root.quit()       # Ensure the mainloop exits
        self.root.destroy()    # Cleanly destroy GUI






class TherapySessionFrame(ttk.Frame):
    def __init__(self, parent, patient_name, shared_state, end_session_callback):
        super().__init__(parent, padding=40)
        self.root = parent  # Required to bind correctly
        self.shared_state = shared_state
        self.end_session_callback = end_session_callback
        self.start_time = time.time()

        # Bind key events globally (safer than per-widget)
        self.root.bind_all("<KeyPress-t>", self._start_listening_mode)
        self.root.bind_all("<KeyRelease-t>", self._stop_listening_mode)

        # Title
        ttk.Label(self, text=f"Therapy Session with {patient_name}",
                  font=("Segoe UI", 20, "bold")).pack(pady=(0, 20))

        # Live Timer
        self.timer_label = ttk.Label(self, text="Session Time: 00:00", font=("Segoe UI", 14))
        self.timer_label.pack(pady=10)
        self._update_timer()

        # Mode Label (optional but useful)
        self.mode_label = ttk.Label(self, text="Mode: idle", font=("Segoe UI", 14, "italic"), foreground="gray")
        self.mode_label.pack(pady=5)
        self._update_mode_label()

        # "Press T to Talk" Notice
        self.talk_label = ttk.Label(
            self,
            text="Hold [T] to Talk to MiRo",
            font=("Segoe UI", 18, "bold"),
            foreground="green"
        )
        self.talk_label.pack(pady=40)

        # End Session Button
        ttk.Button(self, text="End Therapy Session", command=self._end_session,
                   style="Accent.TButton").pack(pady=30, ipady=8)

    def _update_timer(self):
        elapsed = int(time.time() - self.start_time)
        minutes = elapsed // 60
        seconds = elapsed % 60
        self.timer_label.config(text=f"Session Time: {minutes:02d}:{seconds:02d}")
        self.after(1000, self._update_timer)  # Update every second

    def _update_mode_label(self):
        if self.shared_state:
            mode = self.shared_state.get("current_mode", "idle")
            self.mode_label.config(text=f"Mode: {mode}")
        self.after(500, self._update_mode_label)  # Refresh every half second

    def _start_listening_mode(self, event=None):
        if self.shared_state:
            self.shared_state["current_mode"] = "listening"
            print("[TherapySession] T pressed → listening ON")

    def _stop_listening_mode(self, event=None):
        if self.shared_state:
            self.shared_state["current_mode"] = None
            print("[TherapySession] T released → listening OFF")

    def _end_session(self):
        self.root.unbind_all("<KeyPress-t>")
        self.root.unbind_all("<KeyRelease-t>")
        if callable(self.end_session_callback):
            self.end_session_callback()

if __name__ == "__main__":
    patient_data = {
        "name": "Alex Smith",
        "age": "73"
    }

    root = tk.Tk()
    app = ReminiscenceTherapyGUI(root, patient_data)
    root.protocol("WM_DELETE_WINDOW", app._on_app_exit)
    root.mainloop()
