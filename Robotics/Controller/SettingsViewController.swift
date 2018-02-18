//
//  SettingsViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 2/14/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class SettingsViewController: UITableViewController, UITextFieldDelegate {
    
    let ipTextField = UITextField()
    let portTextField = UITextField()
    var updatedServerLocation = false
    
    override func viewDidLoad() {
        super.viewDidLoad()
        tableView.tableFooterView = UIView()
        tableView.isEditing = true
        tableView.allowsSelectionDuringEditing = true
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        if updatedServerLocation {
            self.updatedServerLocation = false
            RequestHandler.instance.connectToServer()
        }
    }
    
    override func numberOfSections(in tableView: UITableView) -> Int {
        return 4
    }
    
    override func tableView(_ tableView: UITableView,
                            titleForHeaderInSection section: Int) -> String? {
        switch section {
        case 0:  return "Server Location"
        case 1:  return "Rooms"
        case 2: return "Info"
        case 3: return "Other"
        default: return nil
        }
    }
    
    override func tableView(_ tableView: UITableView,
                            numberOfRowsInSection section: Int) -> Int {
        switch section {
        case 0:  return 2
        case 1:  return Settings.instance.roomArray.count + 1
        case 2: return 3
        case 3: return 1
        default: return 0
        }
    }
    
    override func tableView(_ tableView: UITableView,
                            canEditRowAt indexPath: IndexPath) -> Bool {
        switch indexPath.section {
        case 1: return indexPath.row != Settings.instance.roomArray.count
        default: return false
        }
    }
    
    override func tableView(_ tableView: UITableView,
                            commit editingStyle: UITableViewCellEditingStyle,
                            forRowAt indexPath: IndexPath) {
        if editingStyle == .delete {
            Settings.instance.roomArray.remove(at: indexPath.row)
            self.tableView.reloadData()
        }
    }
    
    override func tableView(_ tableView: UITableView,
                            cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        if indexPath.section == 0 {
            switch indexPath.row {
            case 0:  let result = UITableViewCell()
                     result.textLabel?.text = "IP: " + Settings.instance.serverIp
                     result.accessoryType = .disclosureIndicator
                     return result
            case 1:  let result = UITableViewCell()
                     result.textLabel?.text = "Port: " + String(Settings.instance.serverPort)
                     result.accessoryType = .disclosureIndicator
                     return result
            default: return UITableViewCell()
            }
        } else if indexPath.section == 1 {
            let result = UITableViewCell()
            if indexPath.row == Settings.instance.roomArray.count {
                result.textLabel?.text = "Add room..."
                result.accessoryView = UIButton(type: .contactAdd)
                result.accessoryView?.isUserInteractionEnabled = false
            } else {
                result.selectionStyle = .none
                result.textLabel?.text = Settings.instance.roomArray[indexPath.row]
            }
            return result
        } else if indexPath.section == 2 {
            let result = UITableViewCell()
            result.selectionStyle = .none
            if indexPath.row == 0 {
                result.textLabel?.text = "University of Washington"
            } else if indexPath.row == 1 {
                result.textLabel?.text = "CSE481c Winter 2018 Team 3"
            } else if indexPath.row == 2 {
                result.textLabel?.text = "Version 0.5"
            }
            return result
        } else if indexPath.section == 3 {
            let result = UITableViewCell()
            result.textLabel?.text = "Speaker mode"
            result.accessoryType = .disclosureIndicator
            return result
        } else {
            fatalError("Unexpected cell requested")
        }
    }
    
    override func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        if indexPath.section == 0 {
            updatedServerLocation = true
            let title: String
            let placeholder: String
            if indexPath.row == 0 {
                title = "Update ip"
                placeholder = Settings.instance.serverIp
            } else {
                // indexPath.row == 1
                title = "Update port"
                placeholder = String(Settings.instance.serverPort)
            }
            
            let alert = UIAlertController(title: title, message: "", preferredStyle: .alert)
            
            alert.addTextField { (textField) in
                textField.text = placeholder
                if indexPath.row == 1 {
                    textField.delegate = self
                }
            }
            
            alert.addAction(UIAlertAction(title: "OK",
                                          style: .default,
                                          handler: { [weak alert] (_) in
                let newVal = alert?.textFields![0].text
                if indexPath.row == 0 {
                    Settings.instance.serverIp = newVal!
                } else {
                    // indexPath.row == 1
                    Settings.instance.serverPort = Int(newVal!)!
                }
                self.tableView.reloadData()
            }))
            
            self.present(alert, animated: true, completion: nil)
        } else if indexPath.section == 1 {
            if indexPath.row == Settings.instance.roomArray.count {
                let alert = UIAlertController(title: "New room", message: "Enter the room name.", preferredStyle: .alert)
                
                alert.addTextField { (textField) in
                    textField.placeholder = "Room name"
                }
                
                alert.addAction(UIAlertAction(title: "OK",
                                              style: .default,
                                              handler: { [weak alert] (_) in
                    let newVal = alert?.textFields![0].text
                    if newVal != "" {
                        Settings.instance.roomArray.append(newVal!)
                        self.tableView.reloadData()
                    }
                }))
                
                alert.addAction(UIAlertAction(title: "Cancel", style: .cancel, handler: { _ in
                    tableView.deselectRow(at: indexPath, animated: false)
                }))
                
                self.present(alert, animated: true, completion: nil)
            } else {
                self.tableView.deselectRow(at: indexPath, animated: false)
            }
        }
    }
    
    override func tableView(_ tableView: UITableView,
                            heightForRowAt indexPath: IndexPath) -> CGFloat {
        return UITableViewAutomaticDimension
    }
    
    // MARK: UITextFieldDelegate
    func textField(_ textField: UITextField,
                   shouldChangeCharactersIn range: NSRange,
                   replacementString string: String) -> Bool {
        let aSet = NSCharacterSet(charactersIn:"0123456789").inverted
        let compSepByCharInSet = string.components(separatedBy: aSet)
        let numberFiltered = compSepByCharInSet.joined(separator: "")
        return string == numberFiltered
    }
}
